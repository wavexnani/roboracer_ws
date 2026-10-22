#!/usr/bin/env python3
"""ROS 2 Node for Adaptive Multi-Regime Hybrid Controller (ARM-HC)."""

import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

from hybrid_controller.track_manager import TrackManager, TrackInfo
from hybrid_controller.hybrid_supervisor import HybridSupervisor, DrivingRegime, HybridControlOutput
from hybrid_controller.visualization import create_raceline_marker, create_regime_marker, create_horizon_marker


class HybridControllerNode(Node):
    """ROS 2 Node orchestrating the Multi-Regime Hybrid Controller."""

    def __init__(self, map_name_override: str = None, mode_override: str = None):
        super().__init__('hybrid_controller_node')

        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('mode', 'Q1')
        self.declare_parameter('low_friction_mode', False)
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('visualize', True)
        self.declare_parameter('rate_hz', 25.0)

        p = lambda name: self.get_parameter(name).value
        raw_map = map_name_override or str(p('map_name'))
        self.map_name = TrackManager.resolve_track_name(raw_map) or raw_map
        self.mode = (mode_override or str(p('mode'))).upper()
        self.low_friction_mode = bool(p('low_friction_mode'))
        self.odom_topic = str(p('odom_topic'))
        self.drive_topic = str(p('drive_topic'))
        self.scan_topic = str(p('scan_topic'))
        self.initialpose_topic = str(p('initialpose_topic'))
        self.visualize = bool(p('visualize'))
        self.rate_hz = float(p('rate_hz'))

        self.get_logger().info(f"Loading track: '{self.map_name}' (Mode: {self.mode})")
        self.track = TrackManager.load_track(
            track_name=self.map_name,
            waypoint_type='raceline',
            low_friction_mode=self.low_friction_mode
        )
        self.supervisor = HybridSupervisor(
            track=self.track,
            mode=self.mode,
            low_friction_mode=self.low_friction_mode,
            mpc_rate_hz=self.rate_hz
        )

        self.latest_odom: Odometry = None
        self.latest_scan: LaserScan = None
        self.e_stop_active = False
        self._first_odom_received = False
        self._init_pose_count = 0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 5)

        if self.visualize:
            self.regime_marker_pub = self.create_publisher(Marker, '/vis/active_regime', 5)
            self.horizon_marker_pub = self.create_publisher(Marker, '/vis/predicted_horizon', 5)
            self.path_marker_pub = self.create_publisher(Marker, '/vis/raceline', 1)
            self.path_marker_pub.publish(create_raceline_marker(self.track.waypoints))

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.estop_sub = self.create_subscription(Bool, '/e_stop', self.estop_callback, 10)

        self.control_timer = self.create_timer(1.0 / self.rate_hz, self.control_loop_tick)
        self.init_pose_timer = self.create_timer(0.5, self._publish_initial_pose_tick)

        self.get_logger().info(f"Hybrid Controller ready! Track: {self.map_name} ({self.track.track_length:.1f}m)")

    def _publish_initial_pose_tick(self):
        if self._first_odom_received:
            self.init_pose_timer.cancel()
            return

        self._init_pose_count += 1
        x0, y0, theta0 = self.track.start_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x0)
        msg.pose.pose.position.y = float(y0)
        msg.pose.pose.orientation.z = math.sin(theta0 / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta0 / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        if not self._first_odom_received:
            self._first_odom_received = True
            if hasattr(self, 'init_pose_timer') and self.init_pose_timer:
                self.init_pose_timer.cancel()
            self.get_logger().info("Simulator connected! Active autonomous driving engaged.")
        self.latest_odom = msg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def estop_callback(self, msg: Bool):
        self.e_stop_active = msg.data
        if self.e_stop_active:
            self.get_logger().warn("EMERGENCY STOP TRIGGERED!")

    def control_loop_tick(self):
        if not self.latest_odom:
            return

        if self.e_stop_active:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.drive_pub.publish(cmd)
            return

        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        cur_x = float(pos.x)
        cur_y = float(pos.y)
        cur_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cur_v = float(math.hypot(self.latest_odom.twist.twist.linear.x, self.latest_odom.twist.twist.linear.y))

        scans_list = list(self.latest_scan.ranges) if self.latest_scan else None
        now_sec = self.get_clock().now().nanoseconds / 1e9

        out: HybridControlOutput = self.supervisor.compute_control(
            cur_x, cur_y, cur_yaw, cur_v, scans=scans_list, sim_time=now_sec
        )

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'ego_racecar/base_link'
        cmd.drive.speed = float(out.speed)
        cmd.drive.steering_angle = float(out.steering)
        self.drive_pub.publish(cmd)

        if self.visualize:
            self.regime_marker_pub.publish(create_regime_marker(out.active_regime, out.speed, out.steering, cur_x, cur_y))
            if out.predicted_horizon is not None:
                self.horizon_marker_pub.publish(create_horizon_marker(out.predicted_horizon))


def main(args=None):
    rclpy.init(args=args)

    map_override = None
    mode_override = None
    for arg in sys.argv[1:]:
        if arg == '--ros-args' or arg.startswith('--') or arg.startswith('__') or ':=' in arg:
            break
        if not arg.startswith('-'):
            if map_override is None:
                map_override = arg
            elif mode_override is None:
                mode_override = arg

    node = HybridControllerNode(map_name_override=map_override, mode_override=mode_override)
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
