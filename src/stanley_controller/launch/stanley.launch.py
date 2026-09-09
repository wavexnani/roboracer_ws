#!/usr/bin/env python3
"""
ROS 2 Launch file for Stanley Controller with dynamic map selection.

Usage:
  ros2 launch stanley_controller stanley.launch.py map:=Austin
  ros2 launch stanley_controller stanley.launch.py map:=Monza waypoint_type:=centerline
  ros2 launch stanley_controller stanley.launch.py map:=Spielberg launch_sim:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='Spielberg',
        description='Name of racetrack map from f1tenth_racetracks (e.g. Austin, Monza, Spielberg, BrandsHatch)'
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

    map_name = LaunchConfiguration('map')
    waypoint_type = LaunchConfiguration('waypoint_type')
    speed_scale = LaunchConfiguration('speed_scale')
    launch_sim = LaunchConfiguration('launch_sim')

    # Stanley controller node
    stanley_node = Node(
        package='stanley_controller',
        executable='stanley_controller_node.py',
        name='stanley_controller_node',
        output='screen',
        parameters=[{
            'map_name': map_name,
            'waypoint_type': waypoint_type,
            'speed_scale': speed_scale,
            'sync_sim_map': True,
            'publish_initial_pose': True
        }]
    )

    # Optional simulator launch
    f1tenth_gym_share = get_package_share_directory('f1tenth_gym_ros')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_gym_share, 'launch', 'gym_bridge_launch.py')
        ),
        condition=IfCondition(launch_sim)
    )

    return LaunchDescription([
        map_arg,
        waypoint_type_arg,
        speed_scale_arg,
        launch_sim_arg,
        stanley_node,
        sim_launch
    ])
