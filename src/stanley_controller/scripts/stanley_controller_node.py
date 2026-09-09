#!/usr/bin/env python3
"""
Stanley Controller ROS 2 Node for F1TENTH / RoboRacer.
Tracks a waypoint raceline at variable speeds using the Stanley steering control law
with exact orthogonal cross-track projection, windowed waypoint tracking, and RViz visualization.
"""

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class StanleyControllerNode(Node):
    """ROS 2 Node for Stanley Path Tracking with Variable Speed."""

    def __init__(self):
        super().__init__('stanley_controller_node')

        # Parameters
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('waypoints_path', '')
        self.declare_parameter('gain_k', 2.3)
        self.declare_parameter('software_k', 0.5)
        self.declare_parameter('gain_ff', 0.12)               # Curvature feedforward gain
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead_dist', 0.15)        # Base lookahead distance [m]
        self.declare_parameter('lookahead_gain', 0.02)        # Speed-proportional lookahead gain [s]
        self.declare_parameter('max_steering_angle', 0.4189)  # ~24 degrees
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('speed_scale', 0.68)
        self.declare_parameter('enable_curvature_speed', True)
        self.declare_parameter('lat_accel_max', 4.0)          # [m/s^2] maximum lateral tire acceleration
        self.declare_parameter('brake_decel', 4.0)            # [m/s^2] braking deceleration for lookahead
        self.declare_parameter('min_lookahead_horizon', 1.0)  # [m] minimum braking horizon distance
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

        # Track progress state
        self._last_idx = 0

        # Load waypoints (x, y, heading psi, target speed)
        self._load_waypoints(str(p('waypoints_path')))

        # ROS 2 Interfaces
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.visualize:
            self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)
            self.path_marker_pub = self.create_publisher(Marker, self.path_marker_topic, 1)
            self._publish_path_marker()

        self.get_logger().info(
            f'Stanley Controller Node ready. Waypoints: {self.num_waypoints}, '
            f'Sub: {self.odom_topic}, Pub: {self.drive_topic}'
        )

    def _load_waypoints(self, path):
        """Loads waypoints from CSV with robust multi-location fallback and comment handling."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            path,
            os.path.join(script_dir, '../waypoints/Spielberg_raceline.csv'),
            os.path.join(script_dir, '../waypoints/Spielberg_centerline.csv'),
            '/sim_ws/src/stanley_controller/waypoints/Spielberg_raceline.csv',
            '/home/yeswanth/roboracer_ws/src/stanley_controller/waypoints/Spielberg_raceline.csv',
            os.path.join(script_dir, '../waypoints/example_waypoints.csv'),
            os.path.join(script_dir, '../../pure_pursuit/waypoints/example_waypoints.csv'),
            '/sim_ws/src/stanley_controller/waypoints/example_waypoints.csv',
            '/home/yeswanth/roboracer_ws/src/stanley_controller/waypoints/example_waypoints.csv',
        ]

        data = None
        used_path = None
        for cand in candidates:
            if cand and os.path.isfile(cand):
                try:
                    # comments='#' correctly skips metadata lines regardless of count
                    data = np.loadtxt(cand, delimiter=';', comments='#')
                    used_path = cand
                    self.get_logger().info(f'Loaded waypoints from: {cand}')
                    break
                except Exception as e:
                    self.get_logger().warn(f'Failed to load {cand}: {e}')

        if data is None:
            raise FileNotFoundError(f'Could not load waypoints from any candidate paths: {candidates}')

        if data.ndim == 1:
            data = data.reshape(1, -1)

        self.waypoints = data[:, [1, 2]]      # Columns 1, 2: x, y [m]
        self.path_headings = data[:, 3]       # Column 3: psi_rad [rad]
        self.num_waypoints = len(self.waypoints)

        # Velocity profile (vx_mps)
        if data.shape[1] > 5:
            self.target_speeds = data[:, 5]   # Column 5: vx_mps [m/s]
        else:
            self.target_speeds = np.full(self.num_waypoints, self.max_speed)

        # Arc length array (s_m) and closed loop track length
        s_col = data[:, 0] if data.shape[1] > 0 else None
        if s_col is not None and len(s_col) > 1 and np.all(np.diff(s_col) > 0.0):
            self.s_arr = s_col
            closure_dist = float(np.hypot(self.waypoints[0, 0] - self.waypoints[-1, 0],
                                          self.waypoints[0, 1] - self.waypoints[-1, 1]))
            self.track_length = float(s_col[-1]) + closure_dist
        else:
            # Fallback: compute cumulative Euclidean arc length from coordinates
            diffs = np.diff(self.waypoints, axis=0)
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            self.s_arr = np.insert(np.cumsum(dists), 0, 0.0)
            closure_dist = float(np.hypot(self.waypoints[0, 0] - self.waypoints[-1, 0],
                                          self.waypoints[0, 1] - self.waypoints[-1, 1]))
            self.track_length = float(self.s_arr[-1]) + closure_dist

        # Curvature profile (kappa_radpm)
        if data.shape[1] > 4 and np.any(np.abs(data[:, 4]) > 1e-5):
            self.kappa = data[:, 4]           # Column 4: kappa_radpm [rad/m]
        else:
            # Fallback: estimate curvature from heading differences over arc length
            dpsi = (np.diff(self.path_headings) + math.pi) % (2.0 * math.pi) - math.pi
            ds = np.diff(self.s_arr)
            ds = np.where(ds < 1e-4, 1e-4, ds)
            k_diff = dpsi / ds
            self.kappa = np.insert(k_diff, -1, k_diff[-1])

    def _corner_speed_limit(self, s_cur, speed):
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

    def _publish_target_marker(self, wpt_x, wpt_y, ref_x, ref_y):
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

        # Dynamic steering modulation: smoothly taper speed up to 25% under hard steering lock
        steer_ratio = abs(steering_angle) / max(self.max_steer, 1e-3)
        if steer_ratio > 0.5:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.5) / 0.5)

        # Heading error safety guard
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
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
