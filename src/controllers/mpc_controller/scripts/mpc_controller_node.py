#!/usr/bin/env python3
"""ROS 2 Node for Model Predictive Control (MPC) Path Tracking."""

import os
import sys
import time
import math
import numpy as np
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

try:
    from mpc_controller.track_manager import TrackManager, TrackInfo
    from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
    from mpc_controller.steering_policy import SteeringPolicyFilter
    from mpc_controller.projection import (
        find_closest_waypoint, project_to_continuous_arc, extract_reference_horizon
    )
    from mpc_controller.speed_profiler import compute_target_velocity
    from mpc_controller.visualization import (
        create_path_marker, create_target_marker, create_horizon_marker
    )
except ImportError:
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    for c in [
        os.path.abspath(os.path.join(cur_dir, '..')),
        '/home/yeswanth/roboracer_ws/src/controllers/mpc_controller',
        '/sim_ws/src/controllers/mpc_controller',
    ]:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)
    from mpc_controller.track_manager import TrackManager, TrackInfo
    from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
    from mpc_controller.steering_policy import SteeringPolicyFilter
    from mpc_controller.projection import (
        find_closest_waypoint, project_to_continuous_arc, extract_reference_horizon
    )
    from mpc_controller.speed_profiler import compute_target_velocity
    from mpc_controller.visualization import (
        create_path_marker, create_target_marker, create_horizon_marker
    )


class MPCControllerNode(Node):
    """ROS 2 Node for Model Predictive Control with Multi-Map Ingestion."""

    def __init__(
        self,
        map_name_override: Optional[str] = None,
        track_override: Optional[TrackInfo] = None,
        **kwargs
    ):
        super().__init__('mpc_controller_node', **kwargs)

        # Track & Map Parameters
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

        # Steering Policy Parameters
        self.declare_parameter('steering_policy', 'RAW')
        self.declare_parameter('policy_hold_threshold_rad', 0.016)
        self.declare_parameter('policy_lattice_quantum_rad', 0.032)
        self.declare_parameter('nominal_actuator_delay_s', 0.020)
        self.declare_parameter('nominal_actuator_slew_rad_s', 3.20)

        # Model Configuration
        self.declare_parameter('model_type', 'yaw_first_order')
        self.declare_parameter('actuator_delay_s', 0.020)

        # Cost Weights
        self.declare_parameter('w_x', 4.0)
        self.declare_parameter('w_y', 4.0)
        self.declare_parameter('w_psi', 2.0)
        self.declare_parameter('w_v', 0.8)
        self.declare_parameter('w_delta', 0.60)
        self.declare_parameter('w_ddelta', 3.0)
        self.declare_parameter('w_a', 0.1)
        self.declare_parameter('steer_ema_alpha', 1.0)
        self.declare_parameter('steer_deadband_rad', 0.0035)

        # Velocity & Safety Limits
        self.declare_parameter('enable_adaptive_speed', True)
        self.declare_parameter('max_straight_speed', 7.5)
        self.declare_parameter('min_corner_speed', 1.8)
        self.declare_parameter('max_lat_accel', 2.5)
        self.declare_parameter('max_accel', 2.5)
        self.declare_parameter('max_decel', 3.2)
        self.declare_parameter('safety_margin_dist', 0.50)

        # Topics & Visualization
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('visualize', True)

        # Read Parameters
        target_map = map_name_override or self.get_parameter('map_name').value
        self.waypoint_type = self.get_parameter('waypoint_type').value
        self.custom_waypoints_path = self.get_parameter('waypoints_path').value
        self.sync_sim_map = self.get_parameter('sync_sim_map').value
        self.publish_initial_pose = self.get_parameter('publish_initial_pose').value
        self.speed_scale = max(0.1, min(1.0, float(self.get_parameter('speed_scale').value)))
        self.visualize = self.get_parameter('visualize').value

        self.steering_policy_name = str(self.get_parameter('steering_policy').value).upper()
        if self.steering_policy_name not in SteeringPolicyFilter.SUPPORTED_POLICIES:
            raise ValueError(f"Unsupported steering_policy '{self.steering_policy_name}'.")

        self.policy_hold_threshold_rad = float(self.get_parameter('policy_hold_threshold_rad').value)
        self.policy_lattice_quantum_rad = float(self.get_parameter('policy_lattice_quantum_rad').value)
        self.nominal_actuator_delay_s = float(self.get_parameter('nominal_actuator_delay_s').value)
        self.nominal_actuator_slew_rad_s = float(self.get_parameter('nominal_actuator_slew_rad_s').value)

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

        # Configure Optimizer
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

        # Steering Policy Filter
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

        # Load Track Waypoints
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

        self.reset(0.0)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        if self.sync_sim_map and TrackManager.sync_sim_yaml(self.track):
            self.get_logger().info(f"Synchronized sim.yaml with map '{self.track.map_path_no_ext}'")

        # ROS 2 Interfaces
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.enable_adaptive_speed:
            self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self._first_odom_received = False
        self._init_pose_count = 0
        if self.publish_initial_pose:
            self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
            self._init_pose_timer = self.create_timer(1.0, self._publish_initial_pose_tick)

        if self.visualize:
            self.target_marker_pub = self.create_publisher(Marker, '/mpc_target_point', 10)
            self.path_marker_pub = self.create_publisher(Marker, '/mpc_reference_path', 1)
            self.mpc_horizon_pub = self.create_publisher(Marker, '/mpc_predicted_horizon', 10)
            self.path_marker_pub.publish(create_path_marker(self.waypoints))

        self.get_logger().info(f"MPC Node ready: track '{self.track.track_name}' ({self.num_waypoints} wpts, N={self.N})")

    def reset(self, initial_steer: float = 0.0) -> None:
        """Reset internal tracking and observer states."""
        self._last_idx = 0
        self._last_control = (0.0, 0.0)
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
        self._last_published_drive_msg = None
        self._last_timing_audit = {}
        self.optimizer.update_weights(w_x=4.0, w_y=4.0, w_psi=2.0, w_v=0.8, w_delta=0.60, w_ddelta=3.0)
        if hasattr(self, 'steering_policy_filter'):
            self.steering_policy_filter.reset(initial_steer)

    def _on_set_parameters(self, params: List[rclpy.parameter.Parameter]) -> SetParametersResult:
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
        msg.pose.pose.orientation.z = math.sin(theta0 / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta0 / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)

    def scan_callback(self, scan_msg: LaserScan):
        self._last_scan = scan_msg

    def odom_callback(self, odom_msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            self.get_logger().info(f"Connected to simulator on '{self.odom_topic}'")

        t_entry = time.perf_counter()
        now_sec = float(odom_msg.header.stamp.sec) + float(odom_msg.header.stamp.nanosec) * 1e-9
        dt_actual = min(0.1, max(0.005, now_sec - self._last_odom_time)) if (self._last_odom_time and now_sec > self._last_odom_time) else 0.04
        self._last_odom_time = now_sec

        # Extract vehicle state
        pos = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        cur_x, cur_y = pos.x, pos.y
        cur_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cur_v = math.hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)
        cur_r = float(odom_msg.twist.twist.angular.z)

        # Detect position jumps
        pos_jump = False
        if self._last_pos is not None and math.hypot(cur_x - self._last_pos[0], cur_y - self._last_pos[1]) > 3.0:
            pos_jump = True
            self.reset(0.0)
        self._last_pos = (cur_x, cur_y)

        # Nearest waypoint & continuous arc-length projection
        closest_idx, _ = find_closest_waypoint(
            cur_x, cur_y, cur_yaw, self.waypoints, self.path_headings,
            self._last_idx, self.num_waypoints, pos_jump
        )
        self._last_idx = closest_idx

        s_cont = project_to_continuous_arc(
            cur_x, cur_y, closest_idx, self.waypoints, self.s_arr,
            self.track_length, self.num_waypoints
        )
        self._last_s_cont = s_cont

        ref_horizon = extract_reference_horizon(
            s_cont, cur_v, self.N, self.dt, self.track_length,
            self.waypoints, self.path_headings, self.s_arr,
            self.scaled_target_speeds, self.num_waypoints
        )

        # Dynamic curvature regime scheduling
        k_lookahead = max(abs(float(self.kappa[(closest_idx + w) % self.num_waypoints])) for w in range(15))
        if self._current_regime == 'CORNER':
            if k_lookahead < 0.032:
                self._current_regime = 'STRAIGHT'
                self.optimizer.update_weights(w_x=4.0, w_y=4.0, w_psi=2.0, w_delta=0.60, w_ddelta=3.0)
        else:
            if k_lookahead >= 0.040:
                self._current_regime = 'CORNER'
                self.optimizer.update_weights(w_x=8.0, w_y=8.0, w_psi=3.2, w_delta=0.20, w_ddelta=1.5)

        heading_err = (self.path_headings[closest_idx] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        recovery_mode = abs(heading_err) > math.radians(55)
        res = None

        if recovery_mode:
            steer_cmd = float(np.clip(heading_err * 1.5, -self.optimizer.cfg.max_steer, self.optimizer.cfg.max_steer))
            target_v = self.min_corner_speed
            self._last_control = (0.0, steer_cmd)
            self._last_speed_cmd = target_v
        else:
            state = np.array([cur_x, cur_y, cur_yaw, cur_v, cur_r]) if self.optimizer.nx == 5 else np.array([cur_x, cur_y, cur_yaw, cur_v])
            res = self.optimizer.solve(state, ref_horizon, self._last_control)
            steer_cmd = float(res.steering)
            self._last_control = (res.accel, res.steering)

            base_speed = float(self.scaled_target_speeds[closest_idx])
            target_v = compute_target_velocity(
                base_speed, steer_cmd, self.optimizer.cfg.max_steer, heading_err,
                self._last_speed_cmd, dt_actual, self.max_accel, self.max_decel,
                self.max_straight_speed, self.min_corner_speed, self._last_scan,
                self.enable_adaptive_speed, self.safety_margin_dist
            )
            self._last_speed_cmd = target_v

        opt_raw_steer = steer_cmd
        t_policy = time.perf_counter()

        if self.steering_policy_name == 'RAW':
            max_dsteer = float(self.optimizer.cfg.max_steer_rate) * dt_actual
            slew_limited = float(np.clip(opt_raw_steer, self._last_steer_cmd - max_dsteer, self._last_steer_cmd + max_dsteer))
            final_cmd = self._last_steer_cmd if abs(slew_limited - self._last_steer_cmd) < self.steer_deadband_rad else slew_limited
            self._last_steer_cmd = final_cmd
            policy_steer = opt_raw_steer
        else:
            policy_steer = self.steering_policy_filter.step(opt_raw_steer)
            self._last_steer_cmd = policy_steer
            final_cmd = policy_steer

        self._filtered_steer = final_cmd
        self._last_opt_raw_steer = opt_raw_steer
        self._last_policy_steer = policy_steer
        self._last_published_steer = final_cmd

        t_pub = time.perf_counter()
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = target_v
        drive_msg.drive.steering_angle = float(self._filtered_steer)
        self.drive_pub.publish(drive_msg)

        self._last_published_drive_msg = drive_msg
        self._last_timing_audit = {'t_entry': t_entry, 't_policy': t_policy, 't_pub': t_pub, 't_sim': now_sec}

        if self.visualize:
            self.target_marker_pub.publish(create_target_marker(ref_horizon[1, 0], ref_horizon[1, 1]))
            if res is not None and len(res.predicted_x) > 0:
                self.mpc_horizon_pub.publish(create_horizon_marker(res.predicted_x, res.predicted_y))


def main(args=None):
    rclpy.init(args=args)

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
