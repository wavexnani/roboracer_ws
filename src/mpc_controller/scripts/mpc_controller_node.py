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
import math
import numpy as np
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

try:
    from mpc_controller.track_manager import TrackManager, TrackInfo
    from mpc_controller.mpc_optimizer import MPCOptimizer, MPCConfig, MPCResult
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


class MPCControllerNode(Node):
    """ROS 2 Node for Model Predictive Control with Multi-Map Ingestion."""

    def __init__(self, map_name_override: Optional[str] = None):
        super().__init__('mpc_controller_node')

        # Parameter declarations
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('waypoint_type', 'raceline')
        self.declare_parameter('waypoints_path', '')
        self.declare_parameter('sync_sim_map', True)
        self.declare_parameter('publish_initial_pose', True)

        # Vehicle & MPC Parameters
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('speed_scale', 0.75)
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.08)

        # MPC Cost Weights
        self.declare_parameter('w_x', 8.0)
        self.declare_parameter('w_y', 8.0)
        self.declare_parameter('w_psi', 3.0)
        self.declare_parameter('w_v', 0.8)
        self.declare_parameter('w_delta', 0.25)
        self.declare_parameter('w_ddelta', 1.5)
        self.declare_parameter('w_a', 0.1)
        self.declare_parameter('steer_ema_alpha', 1.0)
        self.declare_parameter('steer_deadband_rad', 0.0035)  # ~0.20 deg deadband to suppress micro-vibrations

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

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value

        # Build MPC configuration
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
        )
        self.optimizer = MPCOptimizer(mpc_cfg)
        self.N = mpc_cfg.N
        self.dt = mpc_cfg.dt

        # Load Track Waypoints & Profiles
        self.track: TrackInfo = TrackManager.load_track(
            track_name=target_map,
            waypoint_type=self.waypoint_type,
            custom_csv_path=self.custom_waypoints_path if self.custom_waypoints_path else None
        )

        self.waypoints = self.track.waypoints
        self.num_waypoints = len(self.waypoints)
        self.path_headings = self.track.headings
        self.s_arr = self.track.s_arr
        self.kappa = self.track.kappa
        self.scaled_target_speeds = self.track.target_speeds * self.speed_scale
        self.track_length = self.track.track_length

        self._last_idx = 0
        self._last_control = (0.0, 0.0)  # (accel, steer)
        self._last_pos = None  # To detect simulator resets
        self.steer_ema_alpha = float(self.get_parameter('steer_ema_alpha').value)
        self.steer_deadband_rad = float(self.get_parameter('steer_deadband_rad').value)
        self._filtered_steer = 0.0
        self._last_steer_cmd = 0.0

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
        self._last_pos = (cur_x, cur_y)

        # Local window search around last index with heading check
        window_backward = 20
        window_forward = 100
        search_indices = (np.arange(-window_backward, window_forward) + self._last_idx) % self.num_waypoints
        cand_pts = self.waypoints[search_indices]

        dx = cand_pts[:, 0] - cur_x
        dy = cand_pts[:, 1] - cur_y
        dist_sq = dx * dx + dy * dy

        # Heading directional mask: only search waypoints in the forward direction
        dpsi = (self.path_headings[search_indices] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        forward_mask = np.cos(dpsi) > 0.0

        if np.any(forward_mask):
            best_local = int(np.argmin(np.where(forward_mask, dist_sq, np.inf)))
        else:
            best_local = int(np.argmin(dist_sq))

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
                all_cost = np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)
                closest_idx = int(np.argmin(all_cost))
            else:
                closest_idx = int(np.argmin(all_dist_sq))

        self._last_idx = closest_idx

        # Build N+1 horizon references based on cumulative arc length
        ref_horizon = np.zeros((self.N + 1, 4))
        cur_s = self.s_arr[closest_idx]
        speed_est = max(1.5, cur_v)

        for k in range(self.N + 1):
            s_target = (cur_s + k * speed_est * self.dt) % self.track_length
            s_diff = np.abs(self.s_arr - s_target)
            idx_k = int(np.argmin(s_diff))

            ref_horizon[k, 0] = self.waypoints[idx_k, 0]
            ref_horizon[k, 1] = self.waypoints[idx_k, 1]
            ref_horizon[k, 2] = self.path_headings[idx_k]
            ref_horizon[k, 3] = self.scaled_target_speeds[idx_k]

        return ref_horizon, closest_idx

    def odom_callback(self, odom_msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            self.get_logger().info(
                f"Connected to simulator on '{self.odom_topic}'! MPC controller is now actively driving."
            )

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
        current_state = np.array([cur_x, cur_y, cur_yaw, cur_v])

        # 2. Extract reference horizon
        ref_horizon, closest_idx = self._extract_reference_horizon(cur_x, cur_y, cur_yaw, cur_v)

        # 3. Heading check relative to closest path tangent
        heading_err = (self.path_headings[closest_idx] - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # If vehicle spun out or facing backwards (> 85 degrees), apply recovery steering
        wrong_way = abs(heading_err) > math.radians(85)

        if wrong_way:
            # Safe recovery pursuit: steer directly towards path tangent and limit speed
            steer_cmd = float(np.clip(heading_err, -self.optimizer.cfg.max_steer, self.optimizer.cfg.max_steer))
            target_v = 1.0
            self._last_control = (0.0, steer_cmd)
        else:
            # Solve MPC Optimization
            res: MPCResult = self.optimizer.solve(
                current_state=current_state,
                ref_trajectory=ref_horizon,
                prev_control=self._last_control
            )
            steer_cmd = float(res.steering)
            target_v = float(res.target_speed)
            self._last_control = (res.accel, res.steering)

            # Dynamic corner speed modulation: smoothly taper speed up to 25% under hard steering
            steer_ratio = abs(steer_cmd) / max(self.optimizer.cfg.max_steer, 1e-3)
            if steer_ratio > 0.45:
                target_v *= (1.0 - 0.25 * (steer_ratio - 0.45) / 0.55)

            # Heading error safety guard: throttle speed if vehicle diverges from road tangent
            if abs(heading_err) > 0.35:
                target_v *= 0.85

        target_v = float(np.clip(target_v, 0.5, self.scaled_target_speeds[closest_idx]))

        # Micro-deadband filter: suppress sub-0.2 degree servo chatter and tiny vibrations
        if abs(steer_cmd - self._last_steer_cmd) < self.steer_deadband_rad:
            steer_cmd = self._last_steer_cmd
        else:
            self._last_steer_cmd = steer_cmd

        # Smooth commanded steering via 1st-order EMA filter if alpha < 1.0
        if self.steer_ema_alpha < 1.0:
            self._filtered_steer = self.steer_ema_alpha * steer_cmd + (1.0 - self.steer_ema_alpha) * self._filtered_steer
        else:
            self._filtered_steer = steer_cmd

        # 4. Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = target_v
        drive_msg.drive.steering_angle = float(self._filtered_steer)
        self.drive_pub.publish(drive_msg)

        # 5. Visualizations
        if self.visualize:
            # Immediate target marker
            self._publish_target_marker(ref_horizon[1, 0], ref_horizon[1, 1])
            # Green predicted trajectory ribbon
            if not wrong_way and len(res.predicted_x) > 0:
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
