#!/usr/bin/env python3
"""Stanley Controller ROS 2 Node for F1TENTH / RoboRacer."""

import os
import sys
import math
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from stanley_controller.track_manager import TrackManager, TrackInfo
    from stanley_controller.controller import (
        find_reference_waypoint, compute_stanley_steering, compute_corner_speed_limit
    )
    from stanley_controller.visualization import create_path_marker, create_target_marker
except ImportError:
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    for c in [
        os.path.abspath(os.path.join(cur_dir, '..')),
        '/home/yeswanth/roboracer_ws/src/controllers/stanley_controller',
        '/sim_ws/src/controllers/stanley_controller',
    ]:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)
    from stanley_controller.track_manager import TrackManager, TrackInfo
    from stanley_controller.controller import (
        find_reference_waypoint, compute_stanley_steering, compute_corner_speed_limit
    )
    from stanley_controller.visualization import create_path_marker, create_target_marker


class StanleyControllerNode(Node):
    """ROS 2 Node for Stanley Path Tracking with Variable Speed & Multi-Map Support."""

    def __init__(self, map_name_override: Optional[str] = None):
        super().__init__('stanley_controller_node')

        # Parameters
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('waypoint_type', 'raceline')
        self.declare_parameter('waypoints_path', '')
        self.declare_parameter('sync_sim_map', True)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initialpose_topic', '/initialpose')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('gain_k', 2.3)
        self.declare_parameter('software_k', 0.5)
        self.declare_parameter('gain_ff', 0.12)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead_dist', 0.15)
        self.declare_parameter('lookahead_gain', 0.02)
        self.declare_parameter('max_steering_angle', 0.4189)

        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('speed_scale', 0.68)
        self.declare_parameter('enable_curvature_speed', True)
        self.declare_parameter('lat_accel_max', 4.0)
        self.declare_parameter('brake_decel', 4.0)
        self.declare_parameter('min_lookahead_horizon', 1.0)

        self.declare_parameter('visualize', True)
        self.declare_parameter('target_marker_topic', '/vis/stanley_target')
        self.declare_parameter('path_marker_topic', '/vis/stanley_path')

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

        self._last_idx = 0
        requested_map = map_name_override if map_name_override else str(p('map_name'))
        waypoint_type = str(p('waypoint_type'))
        explicit_csv = str(p('waypoints_path')) if p('waypoints_path') else None

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

        if self.sync_sim_map and TrackManager.sync_sim_yaml(self.track):
            self.get_logger().info(f"Synchronized sim.yaml with map: '{self.track.map_path_no_ext}'")

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self._first_odom_received = False
        self._init_pose_count = 0
        if self.publish_initial_pose:
            self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
            self._init_pose_timer = self.create_timer(1.0, self._publish_initial_pose_tick)

        if self.visualize:
            self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)
            self.path_marker_pub = self.create_publisher(Marker, self.path_marker_topic, 1)
            self.path_marker_pub.publish(create_path_marker(self.waypoints))

        self.get_logger().info(f"Stanley Node ready: track '{self.track.track_name}' ({self.num_waypoints} wpts)")

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

    def odom_callback(self, odom_msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, '_init_pose_timer') and self._init_pose_timer:
                self._init_pose_timer.cancel()
            self.get_logger().info(f"Connected to simulator on '{self.odom_topic}'")

        pos = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        current_x, current_y = pos.x, pos.y
        current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        current_vel = math.hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)

        # Front axle reference point with preview
        effective_offset = self.wheelbase + self.lookahead_dist + self.lookahead_gain * current_vel
        ref_x = current_x + effective_offset * math.cos(current_yaw)
        ref_y = current_y + effective_offset * math.sin(current_yaw)

        idx = find_reference_waypoint(
            ref_x, ref_y, current_yaw, self.waypoints, self.path_headings,
            self._last_idx, self.num_waypoints
        )
        self._last_idx = idx

        kappa_val = float(self.kappa[idx]) if self.kappa is not None else None
        steering_angle, _, heading_error = compute_stanley_steering(
            ref_x, ref_y, current_yaw, current_vel,
            self.waypoints[idx, 0], self.waypoints[idx, 1],
            self.path_headings[idx], kappa_val,
            self.gain_k, self.software_k, self.gain_ff, self.wheelbase, self.max_steer
        )

        v_target = float(self.target_speeds[idx]) * self.speed_scale
        if self.enable_curvature_speed:
            s_cur = float(self.s_arr[idx])
            v_corner = compute_corner_speed_limit(
                s_cur, max(v_target, current_vel), self.s_arr, self.kappa,
                self.track_length, self.brake_decel, self.min_lookahead_horizon, self.lat_accel_max
            )
            v_target = min(v_target, v_corner)

        steer_ratio = abs(steering_angle) / max(self.max_steer, 1e-3)
        if steer_ratio > 0.5:
            v_target *= (1.0 - 0.25 * (steer_ratio - 0.5) / 0.5)

        if abs(heading_error) > 0.35:
            v_target *= 0.85

        v_target = float(np.clip(v_target, self.min_speed, self.max_speed))

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'ego_racecar/base_link'
        cmd.drive.speed = v_target
        cmd.drive.steering_angle = steering_angle
        self.drive_pub.publish(cmd)

        if self.visualize:
            self.target_marker_pub.publish(create_target_marker(self.waypoints[idx, 0], self.waypoints[idx, 1]))


def main(args=None):
    rclpy.init(args=args)

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
