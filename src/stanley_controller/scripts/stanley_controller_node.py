#!/usr/bin/env python3
"""
Stanley Controller ROS 2 Node for F1TENTH / RoboRacer.
Tracks a waypoint raceline at variable speeds using the Stanley steering control law
with exact orthogonal cross-track projection, speed-adaptive lookahead, curvature feedforward,
braking-horizon speed adaptation, dynamic map discovery across f1tenth_racetracks, and RViz visualization.
"""

import math
import os
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

try:
    from stanley_controller.track_manager import TrackManager, TrackInfo
except ImportError:
    # Support direct execution without sourcing setup.bash
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    cands = [
        # Source tree paths
        os.path.abspath(os.path.join(cur_dir, '..')),
        os.path.abspath(os.path.join(cur_dir, '../../../../src/stanley_controller')),
        '/sim_ws/src/stanley_controller',
        '/home/yeswanth/roboracer_ws/src/stanley_controller',
        # Installed site-packages paths
        os.path.abspath(os.path.join(cur_dir, f'../../python{sys.version_info.major}.{sys.version_info.minor}/site-packages')),
        '/sim_ws/install/stanley_controller/lib/python3.8/site-packages',
        '/sim_ws/install/stanley_controller/lib/python3.10/site-packages',
        '/sim_ws/install/stanley_controller/lib/python3.12/site-packages',
    ]
    for c in cands:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)
    from stanley_controller.track_manager import TrackManager, TrackInfo


class StanleyControllerNode(Node):
    """ROS 2 Node for Stanley Path Tracking with Variable Speed & Multi-Map Support."""

    def __init__(self, map_name_override: Optional[str] = None):
        super().__init__('stanley_controller_node')

        # Map & Trajectory Profile Parameters
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('waypoint_type', 'raceline')   # 'raceline' or 'centerline'
        self.declare_parameter('waypoints_path', '')          # Optional explicit CSV override
        self.declare_parameter('sync_sim_map', True)          # Automatically update f1tenth_gym sim.yaml
        self.declare_parameter('publish_initial_pose', True)  # Teleport simulator car to map start
        self.declare_parameter('initialpose_topic', '/initialpose')

        # Topics
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')

        # Stanley Control Parameters
        self.declare_parameter('gain_k', 2.3)                 # Cross-track gain
        self.declare_parameter('software_k', 0.5)             # Softening constant (m/s)
        self.declare_parameter('gain_ff', 0.12)               # Curvature feedforward gain
        self.declare_parameter('wheelbase', 0.33)             # Wheelbase L [m]
        self.declare_parameter('lookahead_dist', 0.15)        # Base preview distance ahead of front axle [m]
        self.declare_parameter('lookahead_gain', 0.02)        # Speed-proportional preview gain [s]
        self.declare_parameter('max_steering_angle', 0.4189)  # ~24 degrees [rad]

        # Velocity & Curvature Profile Parameters
        self.declare_parameter('min_speed', 1.0)              # [m/s]
        self.declare_parameter('max_speed', 8.0)              # [m/s]
        self.declare_parameter('speed_scale', 0.68)           # Global velocity scale factor
        self.declare_parameter('enable_curvature_speed', True)
        self.declare_parameter('lat_accel_max', 4.0)          # [m/s^2] maximum lateral tire acceleration
        self.declare_parameter('brake_decel', 4.0)            # [m/s^2] braking deceleration for lookahead
        self.declare_parameter('min_lookahead_horizon', 1.0)  # [m] minimum braking horizon distance

        # Visualization
        self.declare_parameter('visualize', True)
        self.declare_parameter('target_marker_topic', '/vis/stanley_target')
        self.declare_parameter('path_marker_topic', '/vis/stanley_path')

        # Cache parameter values
        p = lambda name: self.get_parameter(name).value
        self.odom_topic = str(p('odom_topic'))
        self.drive_topic = str(p('drive_topic'))
        self.gain_k = float(p('gain_k'))
        self.software_k = float(p('software_k'))
        self.gain_ff = float(p('gain_ff'))
        self.wheelbase = float(p('wheelbase'))
        self.lookahead_dist = float(p('lookahead_dist'))
        self.lookahead_gain = float(p('lookahead_gain'))
        self.max_steer = float(p('max_steering_angle'))
        self.min_speed = float(p('min_speed'))
        self.max_speed = float(p('max_speed'))
        self.speed_scale = float(p('speed_scale'))
        self.enable_curvature_speed = bool(p('enable_curvature_speed'))
        self.lat_accel_max = float(p('lat_accel_max'))
        self.brake_decel = float(p('brake_decel'))
        self.min_lookahead_horizon = float(p('min_lookahead_horizon'))
        self.visualize = bool(p('visualize'))
        self.target_marker_topic = str(p('target_marker_topic'))
        self.path_marker_topic = str(p('path_marker_topic'))
        self.sync_sim_map = bool(p('sync_sim_map'))
        self.publish_initial_pose = bool(p('publish_initial_pose'))
        self.initialpose_topic = str(p('initialpose_topic'))

        # Track progress state
        self._last_idx = 0

        # Determine active map name (CLI override takes precedence over parameter)
        requested_map = map_name_override if map_name_override else str(p('map_name'))
        waypoint_type = str(p('waypoint_type'))
        explicit_csv = str(p('waypoints_path')) if p('waypoints_path') else None

        # Load track geometry and velocity profile via TrackManager
        self.track = TrackManager.load_track(
            track_name=requested_map,
            waypoint_type=waypoint_type,
            custom_csv_path=explicit_csv,
            default_speed=self.max_speed,
            lat_accel_max=self.lat_accel_max
        )

        self.waypoints = self.track.waypoints
        self.path_headings = self.track.headings
        self.s_arr = self.track.s_arr
        self.kappa = self.track.kappa
        self.target_speeds = self.track.target_speeds
        self.track_length = self.track.track_length
        self.num_waypoints = len(self.waypoints)

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
            self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)
            self.path_marker_pub = self.create_publisher(Marker, self.path_marker_topic, 1)
            self._publish_path_marker()

        self.get_logger().info(
            f"Stanley Controller Node initialized for track: '{self.track.track_name}' "
            f"(profile: {self.track.waypoint_type}, {self.num_waypoints} pts, {self.track_length:.1f}m)."
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
                f"or run both together: 'ros2 launch stanley_controller stanley.launch.py map:={self.track.track_name} launch_sim:=true')"
            )

    def _corner_speed_limit(self, s_cur: float, speed: float) -> float:
        """
        Minimum safe cornering speed based on maximum upcoming track curvature
        within a braking-aware lookahead horizon:
        v_corner = sqrt(a_lat_max / kappa_max)
        """
        if self.s_arr is None or self.kappa is None or self.brake_decel <= 0.0:
            return float('inf')

        horizon = (speed * speed) / (2.0 * self.brake_decel) + self.min_lookahead_horizon
        s_rel = (self.s_arr - s_cur) % self.track_length
        mask = s_rel <= horizon

        if not np.any(mask):
            return float('inf')

        k_max = float(np.max(np.abs(self.kappa[mask])))
        if k_max < 1e-4:
            return float('inf')

        return math.sqrt(self.lat_accel_max / k_max)

    def _publish_path_marker(self):
        """Publishes static line strip of the waypoints track for RViz."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'stanley_raceline'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.08  # line width [m]
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

        # Close the loop
        if len(self.waypoints) > 0:
            p = Point()
            p.x = float(self.waypoints[0, 0])
            p.y = float(self.waypoints[0, 1])
            p.z = 0.02
            m.points.append(p)

        self.path_marker_pub.publish(m)

    def _publish_target_marker(self, wpt_x: float, wpt_y: float, ref_x: float, ref_y: float):
        """Publishes sphere marker at target waypoint and reference point."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'stanley_target'
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

    def odom_callback(self, odom_msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            self.get_logger().info(
                f"Connected to simulator on '{self.odom_topic}'! Stanley controller is now actively driving."
            )

        # 1. Extract vehicle state
        pos = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        current_x = pos.x
        current_y = pos.y
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        current_vel = math.hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)

        # 2. Reference point (front axle + speed-adaptive preview distance for corner anticipation)
        effective_offset = self.wheelbase + self.lookahead_dist + self.lookahead_gain * current_vel
        ref_x = current_x + effective_offset * math.cos(current_yaw)
        ref_y = current_y + effective_offset * math.sin(current_yaw)

        # 3. Local window search around last index with heading alignment check
        # This preserves track progression and prevents jumping across hairpins
        window_backward = 20
        window_forward = 100
        search_indices = (np.arange(-window_backward, window_forward) + self._last_idx) % self.num_waypoints
        cand_pts = self.waypoints[search_indices]

        dx = cand_pts[:, 0] - ref_x
        dy = cand_pts[:, 1] - ref_y
        dist_sq = dx * dx + dy * dy

        # Angular difference penalty to prefer waypoints aligned with vehicle direction
        dpsi = (self.path_headings[search_indices] - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
        cost = dist_sq + 4.0 * (dpsi ** 2)

        min_local = int(np.argmin(cost))
        idx = int(search_indices[min_local])

        # Global search fallback if vehicle is far from expected window
        if math.sqrt(dist_sq[min_local]) > 3.0:
            all_dx = self.waypoints[:, 0] - ref_x
            all_dy = self.waypoints[:, 1] - ref_y
            all_dist_sq = all_dx * all_dx + all_dy * all_dy
            all_dpsi = (self.path_headings - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
            valid_mask = np.cos(all_dpsi) > 0.0
            if np.any(valid_mask):
                all_cost = np.where(valid_mask, all_dist_sq + 4.0 * (all_dpsi ** 2), np.inf)
                idx = int(np.argmin(all_cost))
            else:
                idx = int(np.argmin(all_dist_sq))

        self._last_idx = idx

        # 4. Exact continuous orthogonal cross-track error to path tangent
        # Path tangent unit vector at the chosen waypoint
        tx = math.cos(self.path_headings[idx])
        ty = math.sin(self.path_headings[idx])

        # Vector from waypoint to vehicle reference point
        ex = ref_x - self.waypoints[idx, 0]
        ey = ref_y - self.waypoints[idx, 1]

        # Signed 2D orthogonal cross product: tx * ey - ty * ex
        # Positive if reference point is left of path, negative if right
        # Stanley convention: steer right (negative angle) when vehicle is left of path
        lateral_error = tx * ey - ty * ex
        cross_track_error = -lateral_error

        # 5. Heading error normalized to [-pi, pi]
        heading_error = self.path_headings[idx] - current_yaw
        heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi

        # 6. Stanley Control Law with Curvature Feedforward
        cross_track_steering = math.atan2(self.gain_k * cross_track_error, current_vel + self.software_k)
        curvature_feedforward = self.gain_ff * math.atan(self.wheelbase * float(self.kappa[idx])) if self.kappa is not None else 0.0
        steering_angle = heading_error + cross_track_steering + curvature_feedforward
        steering_angle = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        # 7. Target speed with curvature lookahead adaptation and corner stability
        v_target = float(self.target_speeds[idx]) * self.speed_scale

        if self.enable_curvature_speed:
            s_cur = float(self.s_arr[idx])
            v_corner = self._corner_speed_limit(s_cur, max(v_target, current_vel))
            v_target = min(v_target, v_corner)

        # Dynamic steering modulation: smoothly taper speed up to 25% under hard steering lock (U-turns)
        steer_ratio = abs(steering_angle) / max(self.max_steer, 1e-3)
        if steer_ratio > 0.5:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.5) / 0.5)

        # Heading error safety guard: throttle speed if vehicle diverges significantly from road tangent
        if abs(heading_error) > 0.35:
            v_target *= 0.85

        v_target = float(np.clip(v_target, self.min_speed, self.max_speed))

        # 8. Publish Ackermann command
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'ego_racecar/base_link'
        cmd.drive.speed = v_target
        cmd.drive.steering_angle = steering_angle
        self.drive_pub.publish(cmd)

        # 9. Optional RViz target visualization
        if self.visualize:
            self._publish_target_marker(self.waypoints[idx, 0], self.waypoints[idx, 1], ref_x, ref_y)


def main(args=None):
    rclpy.init(args=args)

    # Parse CLI positional argument for map_name (ignoring ROS 2 flags)
    # Examples:
    #   ros2 run stanley_controller stanley_controller_node.py Austin
    #   ros2 run stanley_controller stanley_controller_node.py Monza --ros-args -p speed_scale:=0.65
    map_override = None
    for arg in sys.argv[1:]:
        if arg == '--ros-args' or arg.startswith('--') or arg.startswith('__') or ':=' in arg:
            break
        if not arg.startswith('-'):
            map_override = arg
            break

    node = StanleyControllerNode(map_name_override=map_override)
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
