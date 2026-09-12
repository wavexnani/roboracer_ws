"""
Launch file for Adaptive Multi-Regime Hybrid Controller (hybrid_controller).
ROBORACER IFAC 2026 Regulations Compliant.
Synchronizes track map, start pose, gym simulator, and autofocus camera.
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Ensure hybrid_controller is in sys.path
for p in ['/home/yeswanth/roboracer_ws/src/hybrid_controller', '/sim_ws/src/hybrid_controller']:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from hybrid_controller.track_manager import TrackManager


def launch_setup(context, *args, **kwargs):
    map_name_str = context.perform_substitution(LaunchConfiguration('map'))
    mode_str = context.perform_substitution(LaunchConfiguration('mode')).upper()
    low_friction_str = context.perform_substitution(LaunchConfiguration('low_friction_mode')).lower() == 'true'
    launch_sim_str = context.perform_substitution(LaunchConfiguration('launch_sim')).lower() == 'true'

    track = TrackManager.load_track(
        track_name=map_name_str,
        waypoint_type='raceline',
        low_friction_mode=low_friction_str
    )
    TrackManager.sync_sim_yaml(track)

    actions = []

    # Optional simulator launch
    if launch_sim_str:
        try:
            gym_share = get_package_share_directory('f1tenth_gym_ros')
            gym_launch_path = os.path.join(gym_share, 'launch', 'gym_bridge_launch.py')
            if os.path.isfile(gym_launch_path):
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(gym_launch_path),
                        launch_arguments={'map': track.track_name}.items()
                    )
                )
        except Exception as e:
            print(f"[hybrid.launch.py] Warning: Could not include f1tenth_gym_ros: {e}")

    # Launch Hybrid Controller Node
    actions.append(
        Node(
            package='hybrid_controller',
            executable='hybrid_controller_node.py',
            name='hybrid_controller_node',
            output='screen',
            parameters=[{
                'map_name': track.track_name,
                'mode': mode_str,
                'low_friction_mode': low_friction_str,
                'odom_topic': '/ego_racecar/odom',
                'drive_topic': '/drive',
                'scan_topic': '/scan',
                'initialpose_topic': '/initialpose',
                'visualize': True,
                'rate_hz': 25.0
            }]
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='Spielberg', description='Track name from f1tenth_racetracks'),
        DeclareLaunchArgument('mode', default_value='Q1', description='ROBORACER mission mode: Q1, Q2, Q3, or FINALS'),
        DeclareLaunchArgument('low_friction_mode', default_value='false', description='Low friction urethane concrete mode'),
        DeclareLaunchArgument('launch_sim', default_value='false', description='Launch f1tenth_gym_ros simulator bridge'),
        OpaqueFunction(function=launch_setup)
    ])
