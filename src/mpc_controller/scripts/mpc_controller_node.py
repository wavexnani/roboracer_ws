#!/usr/bin/env python3
"""
ROS 2 Node for Model Predictive Control (MPC) Path Tracking with Multi-Map Support.

Features:
  - Linear Time-Varying (LTV) kinematic bicycle model
  - Fast OSQP Quadratic Program (QP) solver
  - Dynamic multi-map ingestion from f1tenth_racetracks
  - Real-time RViz predicted trajectory horizon visualization
  - Scalable CLI argument and launch file integration

Usage:
  ros2 run mpc_controller mpc_controller_node.py Levine
  ros2 run mpc_controller mpc_controller_node.py Austin --ros-args -p speed_scale:=0.75
"""

import os
import sys
import time
import math
import numpy as np
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

try:
    from mpc_controller.track_manager import TrackManager, TrackInfo
    from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
    from mpc_controller.steering_policy import SteeringPolicyFilter
except ImportError:
    # Support direct execution without sourcing setup.bash
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    cands = [
        # Source tree paths
        os.path.abspath(os.path.join(cur_dir, '..')),
        os.path.abspath(os.path.join(cur_dir, '../../../../src/mpc_controller')),
        '/sim_ws/src/mpc_controller',
        '/home/yeswanth/roboracer_ws/src/mpc_controller',
        # Site-packages paths
        os.path.abspath(os.path.join(cur_dir, f'../../python{sys.version_info.major}.{sys.version_info.minor}/site-packages')),
        '/sim_ws/install/mpc_controller/lib/python3.8/site-packages',
        '/sim_ws/install/mpc_controller/lib/python3.10/site-packages',
        '/sim_ws/install/mpc_controller/lib/python3.12/site-packages',
    ]
    for c in cands:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)
    from mpc_controller.track_manager import TrackManager, TrackInfo
    from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
    from mpc_controller.steering_policy import SteeringPolicyFilter


class MPCControllerNode(Node):
    """ROS 2 Node for Model Predictive Control with Multi-Map Ingestion."""

    def __init__(
        self,
        map_name_override: Optional[str] = None,
        track_override: Optional[TrackInfo] = None,
        **kwargs
    ):
        super().__init__('mpc_controller_node', **kwargs)

        # Parameter declarations
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('waypoint_type', 'raceline')
        self.declare_parameter('waypoints_path', '')
        self.declare_parameter('sync_sim_map', True)
        self.declare_parameter('publish_initial_pose', True)

        # Vehicle & MPC Parameters
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.08)

        # Steering Policy Parameters (Experiment 25B Section 6)
        self.declare_parameter('steering_policy', 'RAW')
        self.declare_parameter('policy_hold_threshold_rad', 0.016)
        self.declare_parameter('policy_lattice_quantum_rad', 0.032)
        self.declare_parameter('nominal_actuator_delay_s', 0.020)
        self.declare_parameter('nominal_actuator_slew_rad_s', 3.20)

        # Validated Predictive Model Configuration (Experiment 11 & 24)
        self.declare_parameter('model_type', 'yaw_first_order')
        self.declare_parameter('actuator_delay_s', 0.020)

        # MPC Cost Weights (Regime-Adaptive Configuration)
        self.declare_parameter('w_x', 4.0)
        self.declare_parameter('w_y', 4.0)
        self.declare_parameter('w_psi', 2.0)
        self.declare_parameter('w_v', 0.8)
        self.declare_parameter('w_delta', 0.60)
        self.declare_parameter('w_ddelta', 3.0)
        self.declare_parameter('w_a', 0.1)
        self.declare_parameter('steer_ema_alpha', 1.0)       # 1.0 = disabled (eliminates filter phase lag)
        self.declare_parameter('steer_deadband_rad', 0.0035)  # 0.0035 rad micro-deadband to suppress straight-line flutter

        # Adaptive Velocity & Safety Parameters
        self.declare_parameter('enable_adaptive_speed', True)
        self.declare_parameter('max_straight_speed', 7.5)
        self.declare_parameter('min_corner_speed', 1.8)
        self.declare_parameter('max_lat_accel', 2.5)
        self.declare_parameter('max_accel', 2.5)
        self.declare_parameter('max_decel', 3.2)
        self.declare_parameter('safety_margin_dist', 0.50)
        self.declare_parameter('scan_topic', '/scan')

        # Topic Names
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('visualize', True)

        # Retrieve parameters
        param_map = self.get_parameter('map_name').value
        target_map = map_name_override or param_map
        self.waypoint_type = self.get_parameter('waypoint_type').value
        self.custom_waypoints_path = self.get_parameter('waypoints_path').value
        self.sync_sim_map = self.get_parameter('sync_sim_map').value
        self.publish_initial_pose = self.get_parameter('publish_initial_pose').value
        self.speed_scale = max(0.1, min(1.0, float(self.get_parameter('speed_scale').value)))
        self.visualize = self.get_parameter('visualize').value

        # Retrieve steering policy parameters
        self.steering_policy_name = str(self.get_parameter('steering_policy').value).upper()
        if self.steering_policy_name not in SteeringPolicyFilter.SUPPORTED_POLICIES:
            raise ValueError(
                f"Unsupported steering_policy '{self.steering_policy_name}'. "
                f"Must be one of {SteeringPolicyFilter.SUPPORTED_POLICIES}"
            )
        self.policy_hold_threshold_rad = float(self.get_parameter('policy_hold_threshold_rad').value)
        self.policy_lattice_quantum_rad = float(self.get_parameter('policy_lattice_quantum_rad').value)
        self.nominal_actuator_delay_s = float(self.get_parameter('nominal_actuator_delay_s').value)
        self.nominal_actuator_slew_rad_s = float(self.get_parameter('nominal_actuator_slew_rad_s').value)

        # Retrieve predictive model configuration
        self.model_type = str(self.get_parameter('model_type').value)
        self.actuator_delay_s = float(self.get_parameter('actuator_delay_s').value)

        self.enable_adaptive_speed = bool(self.get_parameter('enable_adaptive_speed').value)
        self.max_straight_speed = float(self.get_parameter('max_straight_speed').value)
        self.min_corner_speed = float(self.get_parameter('min_corner_speed').value)
        self.max_lat_accel = float(self.get_parameter('max_lat_accel').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.max_decel = float(self.get_parameter('max_decel').value)
        self.safety_margin_dist = float(self.get_parameter('safety_margin_dist').value)
        self.scan_topic = self.get_parameter('scan_topic').value

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value

        # Build MPC configuration with validated model_type and actuator_delay_s
        mpc_cfg = MPCConfig(
            wheelbase=float(self.get_parameter('wheelbase').value),
            dt=float(self.get_parameter('dt').value),
            N=int(self.get_parameter('horizon').value),
            w_x=float(self.get_parameter('w_x').value),
            w_y=float(self.get_parameter('w_y').value),
            w_psi=float(self.get_parameter('w_psi').value),
            w_v=float(self.get_parameter('w_v').value),
            w_delta=float(self.get_parameter('w_delta').value),
            w_ddelta=float(self.get_parameter('w_ddelta').value),
            w_a=float(self.get_parameter('w_a').value),
            model_type=self.model_type,
            actuator_delay_s=self.actuator_delay_s
        )
        self.optimizer = MPCOptimizer(mpc_cfg)
        self.N = mpc_cfg.N
        self.dt = mpc_cfg.dt

        # Command-Side Model-Based Actuator Observer:
        # In the production ROS graph, physical steering angle is NOT measured or published.
        # This filter maintains an internal command-side observer tracking nominal delay and slew.
        self.steering_policy_filter = SteeringPolicyFilter(
            policy=self.steering_policy_name,
            hold_threshold_rad=self.policy_hold_threshold_rad,
            lattice_quantum_rad=self.policy_lattice_quantum_rad,
            nominal_delay_s=self.nominal_actuator_delay_s,
            nominal_slew_rate_rad_s=self.nominal_actuator_slew_rad_s,
            dt_sim_step_s=0.010,
            ctrl_period_s=0.040,
            eps_relay_rad=1e-4
        )

        # Load Track Waypoints & Profiles
        if track_override is not None:
            self.track = track_override
        else:
            self.track = TrackManager.load_track(
                track_name=target_map,
                waypoint_type=self.waypoint_type,
                custom_csv_path=self.custom_waypoints_path if self.custom_waypoints_path else None,
                max_straight_speed=self.max_straight_speed,
                min_corner_speed=self.min_corner_speed,
                lat_accel_max=self.max_lat_accel,
                a_brake_max=self.max_decel,
                a_accel_max=self.max_accel
            )

        self.waypoints = self.track.waypoints
        self.num_waypoints = len(self.waypoints)
        self.path_headings = self.track.headings
        self.s_arr = self.track.s_arr
        self.kappa = self.track.kappa
        self.scaled_target_speeds = self.track.target_speeds * self.speed_scale
        self.track_length = self.track.track_length

        self.steer_ema_alpha = float(self.get_parameter('steer_ema_alpha').value)
        self.steer_deadband_rad = float(self.get_parameter('steer_deadband_rad').value)
        self._last_scan: Optional[LaserScan] = None

        # Telemetry signal holders
        self._last_opt_raw_steer: float = 0.0
        self._last_policy_steer: float = 0.0
        self._last_published_steer: float = 0.0
        self._last_published_drive_msg: Optional[AckermannDriveStamped] = None
        self._last_timing_audit: dict = {}

        # Initialize deterministic tracking and observer state
        self.reset(0.0)

        # Dynamic ROS 2 parameter update callback
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Simulator synchronization
        if self.sync_sim_map:
            synced = TrackManager.sync_sim_yaml(self.track)
            if synced:
                self.get_logger().info(
                    f"Synchronized f1tenth_gym_ros sim.yaml with map: '{self.track.map_path_no_ext}' "
                    f"at start pose {self.track.start_pose}"
                )

        # ROS 2 Interfaces
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.enable_adaptive_speed:
            self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self._first_odom_received = False
        self._init_pose_count = 0
        if self.publish_initial_pose:
            self.initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                self.initialpose_topic,
                10
            )
            # Publish initial pose periodically until simulator odometry is received
            self._init_pose_timer = self.create_timer(1.0, self._publish_initial_pose_tick)

        if self.visualize:
            self.target_marker_pub = self.create_publisher(Marker, '/mpc_target_point', 10)
            self.path_marker_pub = self.create_publisher(Marker, '/mpc_reference_path', 1)
            self.mpc_horizon_pub = self.create_publisher(Marker, '/mpc_predicted_horizon', 10)
            self._publish_path_marker()

        self.get_logger().info(
            f"MPC Controller Node initialized for track: '{self.track.track_name}' "
            f"(profile: {self.track.waypoint_type}, {self.num_waypoints} pts, {self.track_length:.1f}m, N={self.N})."
        )
        self.get_logger().info(
            f"Subscribed to: '{self.odom_topic}' | Publishing to: '{self.drive_topic}'"
        )

    def reset(self, initial_steer: float = 0.0) -> None:
        """
        Resets controller internal tracking state and command-side observer deterministically.

        EQUIVALENCE SEMANTICS (Experiment 25B Section 0 & 8):
        - Nominal SIL Equivalence: initial_steer is strictly 0.0 rad, initializing the observer
          to cur_delta_est = 0.0 and steer_buffer = [0.0, 0.0], matching Experiment 24 reference.
        - Observer state persists across 25-Hz control cycles and is only reset at node
          initialization, when a new run begins, or when a position jump (> 3.0 m) occurs.
        """
        self._last_idx = 0
        self._last_control = (0.0, 0.0)  # (accel, steer)
        self._last_pos = None
        self._last_odom_time = None
        self._last_steer_cmd = float(initial_steer)
        self._last_speed_cmd = 0.0
        self._filtered_steer = float(initial_steer)
        self._last_s_cont = None
        self._current_regime = 'STRAIGHT'
        self._last_opt_raw_steer = float(initial_steer)
        self._last_policy_steer = float(initial_steer)
        self._last_published_steer = float(initial_steer)
        self.optimizer.update_weights(
            w_x=4.0,
            w_y=4.0,
            w_psi=2.0,
            w_v=0.8,
            w_delta=0.60,
            w_ddelta=3.0
        )
        if hasattr(self, 'steering_policy_filter'):
            self.steering_policy_filter.reset(initial_steer)

    def _on_set_parameters(self, params: List[rclpy.parameter.Parameter]) -> SetParametersResult:
        """Dynamically updates node configuration upon ROS parameter changes."""
        for p in params:
            if p.name == 'steering_policy':
                val = str(p.value).upper()
                if val in SteeringPolicyFilter.SUPPORTED_POLICIES:
                    self.steering_policy_name = val
                    if hasattr(self, 'steering_policy_filter'):
                        self.steering_policy_filter.policy = val
                else:
                    return SetParametersResult(successful=False, reason=f"Unsupported steering_policy '{val}'")
            elif p.name == 'policy_hold_threshold_rad':
                self.policy_hold_threshold_rad = float(p.value)
                if hasattr(self, 'steering_policy_filter'):
                    self.steering_policy_filter._hold_threshold = float(p.value)
            elif p.name == 'policy_lattice_quantum_rad':
                self.policy_lattice_quantum_rad = float(p.value)
                if hasattr(self, 'steering_policy_filter'):
                    self.steering_policy_filter._lattice_quantum = float(p.value)
            elif p.name == 'speed_scale':
                self.speed_scale = max(0.1, min(1.0, float(p.value)))
                if hasattr(self, 'track'):
                    self.scaled_target_speeds = self.track.target_speeds * self.speed_scale
        return SetParametersResult(successful=True)

    def _publish_initial_pose_tick(self):
        """Periodically publishes initial pose until simulator connects."""
        if self._first_odom_received:
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            return

        self._init_pose_count += 1
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        x0, y0, theta0 = self.track.start_pose
        msg.pose.pose.position.x = float(x0)
        msg.pose.pose.position.y = float(y0)
        msg.pose.pose.position.z = 0.0

        # Planar yaw quaternion (rotation around Z axis)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta0 / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta0 / 2.0)

        # Standard covariance matrix
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.initial_pose_pub.publish(msg)

        if self._init_pose_count == 1:
            self.get_logger().info(
                f"Published initial pose to {self.initialpose_topic}: "
                f"x={x0:.3f}, y={y0:.3f}, yaw={math.degrees(theta0):.1f}°"
            )
        elif self._init_pose_count == 3:
            self.get_logger().warn(
                f"Waiting for simulator odometry on '{self.odom_topic}'... "
                f"(If simulator is not running, run in another terminal: "
                f"'ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:={self.track.track_name}' "
                f"or run both together: 'ros2 launch mpc_controller mpc.launch.py map:={self.track.track_name} launch_sim:=true')"
            )

    def _extract_reference_horizon(
        self,
        cur_x: float,
        cur_y: float,
        cur_yaw: float,
        cur_v: float
    ) -> Tuple[np.ndarray, int]:
        """
        Extracts N+1 reference states along the trajectory starting from the
        closest point forward, spaced by approximately (v * dt).
        Includes global search fallback when vehicle is far or reset.
        """
        # Detect sudden position jump (e.g. simulator reset via /initialpose)
        pos_jump = False
        if self._last_pos is not None:
            if math.hypot(cur_x - self._last_pos[0], cur_y - self._last_pos[1]) > 3.0:
                pos_jump = True
                self.reset(0.0)
        self._last_pos = (cur_x, cur_y)

        # Monotonic forward search around last index with forward bias
        window_backward = 15
        window_forward = 80
        search_offsets = np.arange(-window_backward, window_forward)
        search_indices = (search_offsets + self._last_idx) % self.num_waypoints
        cand_pts = self.waypoints[search_indices]

        dx = cand_pts[:, 0] - cur_x
        dy = cand_pts[:, 1] - cur_y
        dist_sq = dx * dx + dy * dy

        # Heading directional mask: only search waypoints facing forward (within ~80 deg)
        dpsi = (self.path_headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        forward_mask = np.cos(dpsi) > 0.17

        # Slight backward penalty to favor forward movement along raceline
        index_bias = np.where(search_offsets < 0, 0.4 * (-search_offsets), 0.0)
        cost = dist_sq + 2.0 * (dpsi ** 2) + index_bias

        if np.any(forward_mask):
            best_local = int(np.argmin(np.where(forward_mask, cost, np.inf)))
        else:
            best_local = int(np.argmin(cost))

        closest_idx = int(search_indices[best_local])
        min_dist = math.sqrt(dist_sq[best_local])

        # Global search fallback if vehicle is far from expected window or on reset
        if min_dist > 3.0 or pos_jump:
            all_dx = self.waypoints[:, 0] - cur_x
            all_dy = self.waypoints[:, 1] - cur_y
            all_dist_sq = all_dx * all_dx + all_dy * all_dy
            all_dpsi = (self.path_headings - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
            valid_mask = np.cos(all_dpsi) > 0.0
            if np.any(valid_mask):
                closest_idx = int(np.argmin(np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)))
            else:
                closest_idx = int(np.argmin(all_dist_sq))

        self._last_idx = closest_idx

        # Determine closest track segment robustly using offsets [-1, 0, 1] relative to closest_idx
        cand_offsets = [-1, 0, 1]
        best_dist_sq = float('inf')
        best_seg_a = closest_idx
        best_seg_b = (closest_idx + 1) % self.num_waypoints
        best_t = 0.0
        best_s_cont = self.s_arr[closest_idx]

        p = np.array([cur_x, cur_y], dtype=np.float64)

        for offset in cand_offsets:
            ia = (closest_idx + offset) % self.num_waypoints
            ib = (ia + 1) % self.num_waypoints
            pa = self.waypoints[ia]
            pb = self.waypoints[ib]
            v = pb - pa
            L2 = float(np.dot(v, v))
            if L2 < 1e-12:
                t = 0.0
            else:
                t = float(np.clip(np.dot(p - pa, v) / L2, 0.0, 1.0))
            p_proj = pa + t * v
            dist_sq = float(np.dot(p - p_proj, p - p_proj))

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_seg_a = ia
                best_seg_b = ib
                best_t = t
                if ia == self.num_waypoints - 1:
                    seg_len = self.track_length - self.s_arr[ia]
                else:
                    seg_len = self.s_arr[ib] - self.s_arr[ia]
                best_s_cont = (self.s_arr[ia] + t * seg_len) % self.track_length

        s_cont = best_s_cont

        # Diagnostics and assertions for projection integrity
        assert np.isfinite(s_cont), f"s_cont is not finite: {s_cont}"
        assert 0.0 <= s_cont < self.track_length, f"s_cont={s_cont} outside [0, {self.track_length})"
        assert 0.0 <= best_t <= 1.0, f"Projection parameter t={best_t} outside [0, 1]"

        # Track and log progress between cycles, flagging anomalous backward motion without hard failure
        if hasattr(self, '_last_s_cont') and self._last_s_cont is not None and not pos_jump:
            ds_progress = s_cont - self._last_s_cont
            if ds_progress < -self.track_length / 2.0:
                ds_progress += self.track_length
            elif ds_progress > self.track_length / 2.0:
                ds_progress -= self.track_length
            if ds_progress < -0.20:
                self.get_logger().warn(
                    f"Anomalous backward s_cont motion: ds={ds_progress:.4f}m (from {self._last_s_cont:.4f} to {s_cont:.4f})"
                )
        self._last_s_cont = s_cont

        # Build N+1 horizon references anchored at continuous s_cont
        ref_horizon = np.zeros((self.N + 1, 4))
        cur_s = s_cont
        speed_est = max(1.5, cur_v)
        s_targets = np.zeros(self.N + 1)

        for k in range(self.N + 1):
            s_target = (cur_s + k * speed_est * self.dt) % self.track_length
            s_targets[k] = s_target

            # Find segment [idx_a, idx_b] containing s_target
            idx_a = int(np.searchsorted(self.s_arr, s_target, side='right') - 1)
            idx_a = max(0, min(self.num_waypoints - 1, idx_a))
            idx_b = (idx_a + 1) % self.num_waypoints

            s_a = self.s_arr[idx_a]
            if idx_a == self.num_waypoints - 1:
                # Segment bridging last waypoint and first waypoint across start/finish
                seg_len = self.track_length - s_a
                ds = s_target - s_a if s_target >= s_a else (s_target + self.track_length - s_a)
            else:
                s_b = self.s_arr[idx_b]
                seg_len = s_b - s_a
                ds = s_target - s_a

            t = float(np.clip(ds / max(seg_len, 1e-4), 0.0, 1.0))

            # Continuous linear interpolation of position
            ref_horizon[k, 0] = (1.0 - t) * self.waypoints[idx_a, 0] + t * self.waypoints[idx_b, 0]
            ref_horizon[k, 1] = (1.0 - t) * self.waypoints[idx_a, 1] + t * self.waypoints[idx_b, 1]

            # Shortest-arc angular interpolation of heading
            dpsi_seg = (self.path_headings[idx_b] - self.path_headings[idx_a] + math.pi) % (2.0 * math.pi) - math.pi
            ref_horizon[k, 2] = (self.path_headings[idx_a] + t * dpsi_seg + math.pi) % (2.0 * math.pi) - math.pi

            # Continuous linear interpolation of target speed
            ref_horizon[k, 3] = (1.0 - t) * self.scaled_target_speeds[idx_a] + t * self.scaled_target_speeds[idx_b]

        # Verify reference horizon integrity
        assert abs(s_targets[0] - s_cont) < 1e-9, f"s_ref[0] mismatch: {s_targets[0]} vs {s_cont}"
        for k in range(self.N):
            ds_k = (s_targets[k + 1] - s_targets[k] + self.track_length) % self.track_length
            assert ds_k > 0.0, f"Reference horizon arc-length non-monotonic at step {k}: ds={ds_k}"
        assert np.all(np.isfinite(ref_horizon)), "ref_horizon contains NaN or Inf"

        return ref_horizon, closest_idx

    def scan_callback(self, scan_msg: LaserScan):
        """Stores latest LiDAR scan for forward safety corridor and obstacle analysis."""
        self._last_scan = scan_msg

    def odom_callback(self, odom_msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            self.get_logger().info(
                f"Connected to simulator on '{self.odom_topic}'! MPC controller is now actively driving."
            )

        t_entry = time.perf_counter()

        # Determine actual dt between consecutive callbacks for accurate slew limiting
        now_sec = float(odom_msg.header.stamp.sec) + float(odom_msg.header.stamp.nanosec) * 1e-9
        if self._last_odom_time is not None and now_sec > self._last_odom_time:
            dt_actual = min(0.1, max(0.005, now_sec - self._last_odom_time))
        else:
            dt_actual = 0.04
        self._last_odom_time = now_sec

        # 1. Extract vehicle state
        pos = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        cur_x = pos.x
        cur_y = pos.y
        cur_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        cur_v = math.hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)
        cur_r = float(odom_msg.twist.twist.angular.z)

        if self.optimizer.nx == 5:
            current_state = np.array([cur_x, cur_y, cur_yaw, cur_v, cur_r])
        else:
            current_state = np.array([cur_x, cur_y, cur_yaw, cur_v])

        # 2. Extract reference horizon
        ref_horizon, closest_idx = self._extract_reference_horizon(cur_x, cur_y, cur_yaw, cur_v)

        # 3. Dynamic curvature lookahead for regime-adaptive weight scheduling
        # Evaluates peak curvature across upcoming ~15 waypoints (~3 meters ahead)
        k_lookahead = max(abs(float(self.kappa[(closest_idx + w) % self.num_waypoints])) for w in range(15))

        # Regime classification with hysteresis
        if self._current_regime == 'CORNER':
            if k_lookahead < 0.032:
                self._current_regime = 'STRAIGHT'
                self.optimizer.update_weights(w_x=4.0, w_y=4.0, w_psi=2.0, w_delta=0.60, w_ddelta=3.0)
        else:
            if k_lookahead >= 0.040:
                self._current_regime = 'CORNER'
                self.optimizer.update_weights(w_x=8.0, w_y=8.0, w_psi=3.2, w_delta=0.20, w_ddelta=1.5)

        # 4. Heading check relative to closest path tangent
        heading_err = (self.path_headings[closest_idx] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # If vehicle spun out or facing backwards (> 55 degrees), apply recovery steering
        recovery_mode = abs(heading_err) > math.radians(55)

        if recovery_mode:
            # Safe recovery pursuit: steer directly towards path tangent and limit speed
            steer_cmd = float(np.clip(heading_err * 1.5, -self.optimizer.cfg.max_steer, self.optimizer.cfg.max_steer))
            target_v = self.min_corner_speed
            self._last_control = (0.0, steer_cmd)
            self._last_speed_cmd = target_v
        else:
            # Solve MPC Optimization
            res: MPCResult = self.optimizer.solve(
                current_state=current_state,
                ref_trajectory=ref_horizon,
                prev_control=self._last_control
            )
            steer_cmd = float(res.steering)
            self._last_control = (res.accel, res.steering)

            # 1. Base target speed from pre-braked curvature lookahead profile
            v_target = float(self.scaled_target_speeds[closest_idx])

            # 2. Dynamic LiDAR obstacle & Cartesian forward corridor evaluation
            if self.enable_adaptive_speed and self._last_scan is not None:
                ranges = np.array(self._last_scan.ranges)
                n_beams = len(ranges)
                angle_min = self._last_scan.angle_min
                angle_inc = self._last_scan.angle_increment if self._last_scan.angle_increment > 0 else (4.7 / max(n_beams, 1))
                # Sensor beams fixed in base_link robot chassis frame
                angles = angle_min + np.arange(n_beams) * angle_inc
                valid = np.isfinite(ranges) & (ranges >= max(0.10, self._last_scan.range_min)) & (ranges <= min(25.0, self._last_scan.range_max))
                if np.any(valid):
                    r_val = ranges[valid]
                    th_val = angles[valid]
                    x_body = r_val * np.cos(th_val)
                    y_body = r_val * np.sin(th_val)
                    # Driving corridor directly in front of the vehicle (+/- 0.28m lateral)
                    corridor = (x_body > 0.35) & (x_body < 10.0) & (np.abs(y_body) <= 0.28)
                    if np.any(corridor):
                        d_obs = float(np.min(x_body[corridor]))
                        v_obs = math.sqrt(2.0 * self.max_decel * max(0.0, d_obs - self.safety_margin_dist))
                        v_target = min(v_target, max(self.min_corner_speed, v_obs))

                    # Proximity safety: if very close to obstacle or wall (< 0.35m), scale down
                    min_scan_all = float(np.min(ranges[valid]))
                    if min_scan_all < 0.35:
                        v_target *= max(0.70, min_scan_all / 0.35)

            # 3. Dynamic corner speed modulation: smoothly taper speed up to 25% under hard steering
            steer_ratio = abs(steer_cmd) / max(self.optimizer.cfg.max_steer, 1e-3)
            if steer_ratio > 0.45:
                v_target *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)

            # 4. Heading error safety guard: throttle speed if vehicle diverges from road tangent
            if abs(heading_err) > 0.35:
                v_target *= 0.85

            # 5. Smooth longitudinal rate-limiting using true dt
            v_cmd_max = self._last_speed_cmd + self.max_accel * dt_actual
            v_cmd_min = self._last_speed_cmd - self.max_decel * dt_actual
            if self._last_speed_cmd < 0.2 and v_target > 0.5:
                v_cmd_max = max(v_cmd_max, 0.8)

            target_v = float(np.clip(v_target, max(0.0, v_cmd_min), min(self.max_straight_speed, v_cmd_max)))
            self._last_speed_cmd = target_v

        opt_raw_steer = steer_cmd

        # ----------------------------------------------------------------------
        # STEERING COMMAND TRANSFORMATION (Experiment 25B Sections 4, 5, 7)
        # ----------------------------------------------------------------------
        t_policy = time.perf_counter()

        if self.steering_policy_name == 'RAW':
            # Existing production fallback path: 2.0 rad/s limiter + 3.5 mrad deadband
            max_steer_rate = float(self.optimizer.cfg.max_steer_rate)  # 2.0 rad/s
            max_dsteer = max_steer_rate * dt_actual
            steer_slew_limited = float(np.clip(opt_raw_steer, self._last_steer_cmd - max_dsteer, self._last_steer_cmd + max_dsteer))

            if abs(steer_slew_limited - self._last_steer_cmd) < self.steer_deadband_rad:
                steer_final = self._last_steer_cmd
            else:
                steer_final = steer_slew_limited
                self._last_steer_cmd = steer_final

            policy_steer = opt_raw_steer
            final_cmd = steer_final
        else:
            # Terminal steering-command transformation: replaces limiter and deadband.
            # Downstream clipping or deadbanding is strictly prohibited to prevent destroying the lattice.
            policy_steer = self.steering_policy_filter.step(opt_raw_steer)
            self._last_steer_cmd = policy_steer
            final_cmd = policy_steer

        self._filtered_steer = final_cmd

        # Retain separate telemetry signals
        self._last_opt_raw_steer = opt_raw_steer
        self._last_policy_steer = policy_steer
        self._last_published_steer = final_cmd

        # 4. Publish drive command
        t_pub = time.perf_counter()
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = target_v
        drive_msg.drive.steering_angle = float(self._filtered_steer)
        self.drive_pub.publish(drive_msg)

        self._last_published_drive_msg = drive_msg
        self._last_timing_audit = {
            't_entry': t_entry,
            't_policy': t_policy,
            't_pub': t_pub,
            't_sim': now_sec
        }

        # 5. Visualizations
        if self.visualize:
            # Immediate target marker
            self._publish_target_marker(ref_horizon[1, 0], ref_horizon[1, 1])
            # Green predicted trajectory ribbon
            if not recovery_mode and len(res.predicted_x) > 0:
                self._publish_horizon_marker(res.predicted_x, res.predicted_y)

    def _publish_path_marker(self):
        """Publishes static line strip of track waypoints for RViz."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpc_reference_path'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.7

        for pt in self.waypoints:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.02
            m.points.append(p)

        if len(self.waypoints) > 0:
            p = Point()
            p.x = float(self.waypoints[0, 0])
            p.y = float(self.waypoints[0, 1])
            p.z = 0.02
            m.points.append(p)

        self.path_marker_pub.publish(m)

    def _publish_target_marker(self, wpt_x: float, wpt_y: float):
        """Publishes red sphere marker at lookahead target."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpc_target'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(wpt_x)
        m.pose.position.y = float(wpt_y)
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 0.35
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.0
        m.color.a = 0.95
        self.target_marker_pub.publish(m)

    def _publish_horizon_marker(self, pred_x: np.ndarray, pred_y: np.ndarray):
        """Publishes bright green line strip of predicted MPC trajectory in RViz."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpc_predicted_horizon'
        m.id = 2
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.12  # thickness
        m.color.r = 0.1
        m.color.g = 1.0
        m.color.b = 0.1
        m.color.a = 0.95

        for i in range(len(pred_x)):
            p = Point()
            p.x = float(pred_x[i])
            p.y = float(pred_y[i])
            p.z = 0.05
            m.points.append(p)

        self.mpc_horizon_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)

    # Detect map name passed as CLI positional argument (ignoring ROS 2 flags)
    # Examples:
    #   ros2 run mpc_controller mpc_controller_node.py Austin
    #   ros2 run mpc_controller mpc_controller_node.py Monza --ros-args -p speed_scale:=0.65
    map_name_override = None
    for arg in sys.argv[1:]:
        if arg == '--ros-args' or arg.startswith('--') or arg.startswith('__') or ':=' in arg:
            break
        if not arg.startswith('-'):
            map_name_override = arg
            break

    node = MPCControllerNode(map_name_override=map_name_override)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
