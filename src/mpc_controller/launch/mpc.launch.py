#!/usr/bin/env python3
"""
ROS 2 Launch file for Model Predictive Controller (MPC) with dynamic map selection.

Usage:
  ros2 launch mpc_controller mpc.launch.py map:=Levine launch_sim:=true
  ros2 launch mpc_controller mpc.launch.py map:=Austin launch_sim:=true
  ros2 launch mpc_controller mpc.launch.py map:=Monza waypoint_type:=centerline launch_sim:=true
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    map_str = context.launch_configurations.get('map', 'Spielberg').strip()
    waypoint_type_str = context.launch_configurations.get('waypoint_type', 'raceline').strip()
    speed_scale_str = context.launch_configurations.get('speed_scale', '1.0').strip()
    max_speed_str = context.launch_configurations.get('max_speed', '7.5').strip()
    min_speed_str = context.launch_configurations.get('min_speed', '1.8').strip()
    max_lat_accel_str = context.launch_configurations.get('max_lat_accel', '2.5').strip()
    steer_deadband_str = context.launch_configurations.get('steer_deadband', '0.0035').strip()
    scan_topic_str = context.launch_configurations.get('scan_topic', '/scan').strip()
    launch_sim_bool = context.launch_configurations.get('launch_sim', 'false').strip().lower() in ('true', '1')
    autofocus_str = context.launch_configurations.get('autofocus', 'true').strip()

    # Pre-sync sim.yaml using TrackManager before launching nodes
    for p in ['/sim_ws/src/mpc_controller', '/home/yeswanth/roboracer_ws/src/mpc_controller']:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    try:
        from mpc_controller.track_manager import TrackManager
        track_info = TrackManager.load_track(
            map_str,
            waypoint_type=waypoint_type_str,
            max_straight_speed=float(max_speed_str),
            min_corner_speed=float(min_speed_str),
            lat_accel_max=float(max_lat_accel_str)
        )
        TrackManager.sync_sim_yaml(track_info)
        canonical_map = track_info.track_name
    except Exception as e:
        print(f"[mpc.launch] Warning pre-syncing track '{map_str}': {e}")
        canonical_map = map_str

    # MPC controller node
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_controller_node.py',
        name='mpc_controller_node',
        output='screen',
        parameters=[{
            'map_name': canonical_map,
            'waypoint_type': waypoint_type_str,
            'speed_scale': float(speed_scale_str),
            'max_straight_speed': float(max_speed_str),
            'min_corner_speed': float(min_speed_str),
            'max_lat_accel': float(max_lat_accel_str),
            'steer_deadband_rad': float(steer_deadband_str),
            'scan_topic': scan_topic_str,
            'sync_sim_map': True,
            'publish_initial_pose': True
        }]
    )

    actions = [mpc_node]

    # Optional simulator launch
    if launch_sim_bool:
        try:
            f1tenth_gym_share = get_package_share_directory('f1tenth_gym_ros')
            sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_gym_share, 'launch', 'gym_bridge_launch.py')
                ),
                launch_arguments={
                    'map': canonical_map,
                    'autofocus': autofocus_str
                }.items()
            )
            actions.append(sim_launch)
        except Exception as e:
            print(f"[mpc.launch] Warning including gym_bridge_launch: {e}")

    return actions


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='Spielberg',
        description='Name of racetrack map from f1tenth_racetracks (e.g. Austin, Monza, Spielberg, Levine, BrandsHatch)'
    )
    waypoint_type_arg = DeclareLaunchArgument(
        'waypoint_type',
        default_value='raceline',
        description='Trajectory profile type: raceline or centerline'
    )
    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale',
        default_value='1.0',
        description='Speed scaling factor (0.1 to 1.0)'
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='7.5',
        description='Maximum straightaway speed [m/s] when path is clear'
    )
    min_speed_arg = DeclareLaunchArgument(
        'min_speed',
        default_value='1.8',
        description='Minimum cornering speed [m/s] in hairpins'
    )
    max_lat_accel_arg = DeclareLaunchArgument(
        'max_lat_accel',
        default_value='2.5',
        description='Maximum lateral acceleration [m/s^2] in corners'
    )
    steer_deadband_arg = DeclareLaunchArgument(
        'steer_deadband',
        default_value='0.0035',
        description='Steering deadband threshold [rad] to eliminate micro-vibrations (~0.20 deg)'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic name for obstacle detection'
    )
    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim',
        default_value='false',
        description='Whether to also launch the f1tenth_gym_ros simulation bridge'
    )
    autofocus_arg = DeclareLaunchArgument(
        'autofocus',
        default_value='true',
        description='Whether RViz camera auto-focuses on the car (true) or displays full static map (false)'
    )

    return LaunchDescription([
        map_arg,
        waypoint_type_arg,
        speed_scale_arg,
        max_speed_arg,
        min_speed_arg,
        max_lat_accel_arg,
        steer_deadband_arg,
        scan_topic_arg,
        launch_sim_arg,
        autofocus_arg,
        OpaqueFunction(function=launch_setup)
    ])
