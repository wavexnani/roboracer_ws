#!/usr/bin/env python3
"""
ROS 2 Launch file for Stanley Controller with dynamic map selection.

Usage:
  ros2 launch stanley_controller stanley.launch.py map:=Austin
  ros2 launch stanley_controller stanley.launch.py map:=Monza waypoint_type:=centerline
  ros2 launch stanley_controller stanley.launch.py map:=Spielberg launch_sim:=true
  ros2 launch stanley_controller stanley.launch.py map:=Levine launch_sim:=true
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
    speed_scale_str = context.launch_configurations.get('speed_scale', '0.68').strip()
    launch_sim_bool = context.launch_configurations.get('launch_sim', 'false').strip().lower() in ('true', '1')

    # Pre-sync sim.yaml using TrackManager before launching nodes
    for p in ['/sim_ws/src/stanley_controller', '/home/yeswanth/roboracer_ws/src/stanley_controller']:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    try:
        from stanley_controller.track_manager import TrackManager
        track_info = TrackManager.load_track(map_str, waypoint_type=waypoint_type_str)
        TrackManager.sync_sim_yaml(track_info)
        canonical_map = track_info.track_name
    except Exception as e:
        print(f"[stanley.launch] Warning pre-syncing track '{map_str}': {e}")
        canonical_map = map_str

    # Stanley controller node
    stanley_node = Node(
        package='stanley_controller',
        executable='stanley_controller_node.py',
        name='stanley_controller_node',
        output='screen',
        parameters=[{
            'map_name': canonical_map,
            'waypoint_type': waypoint_type_str,
            'speed_scale': float(speed_scale_str),
            'sync_sim_map': True,
            'publish_initial_pose': True
        }]
    )

    actions = [stanley_node]

    # Optional simulator launch
    if launch_sim_bool:
        try:
            f1tenth_gym_share = get_package_share_directory('f1tenth_gym_ros')
            sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_gym_share, 'launch', 'gym_bridge_launch.py')
                ),
                launch_arguments={'map': canonical_map}.items()
            )
            actions.append(sim_launch)
        except Exception as e:
            print(f"[stanley.launch] Warning including gym_bridge_launch: {e}")

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
        default_value='0.68',
        description='Speed scaling factor (0.1 to 1.0)'
    )
    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim',
        default_value='false',
        description='Whether to also launch the f1tenth_gym_ros simulation bridge'
    )

    return LaunchDescription([
        map_arg,
        waypoint_type_arg,
        speed_scale_arg,
        launch_sim_arg,
        OpaqueFunction(function=launch_setup)
    ])
