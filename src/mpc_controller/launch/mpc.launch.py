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
    steering_policy_str = context.launch_configurations.get('steering_policy', 'RAW').strip()
    policy_hold_threshold_str = context.launch_configurations.get('policy_hold_threshold_rad', '0.016').strip()
    policy_lattice_quantum_str = context.launch_configurations.get('policy_lattice_quantum_rad', '0.032').strip()
    nominal_actuator_delay_str = context.launch_configurations.get('nominal_actuator_delay_s', '0.020').strip()
    nominal_actuator_slew_str = context.launch_configurations.get('nominal_actuator_slew_rad_s', '3.20').strip()
    model_type_str = context.launch_configurations.get('model_type', 'yaw_first_order').strip()
    actuator_delay_s_str = context.launch_configurations.get('actuator_delay_s', '0.020').strip()
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
            'steering_policy': steering_policy_str,
            'policy_hold_threshold_rad': float(policy_hold_threshold_str),
            'policy_lattice_quantum_rad': float(policy_lattice_quantum_str),
            'nominal_actuator_delay_s': float(nominal_actuator_delay_str),
            'nominal_actuator_slew_rad_s': float(nominal_actuator_slew_str),
            'model_type': model_type_str,
            'actuator_delay_s': float(actuator_delay_s_str),
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
    steering_policy_arg = DeclareLaunchArgument(
        'steering_policy',
        default_value='RAW',
        description='Steering policy to apply before publication (RAW, HOLD16, HOLD_LATTICE16)'
    )
    policy_hold_threshold_arg = DeclareLaunchArgument(
        'policy_hold_threshold_rad',
        default_value='0.016',
        description='Policy hold deadband threshold radius [rad] (default: 0.016)'
    )
    policy_lattice_quantum_arg = DeclareLaunchArgument(
        'policy_lattice_quantum_rad',
        default_value='0.032',
        description='Policy lattice quantum grid spacing [rad] (default: 0.032)'
    )
    nominal_actuator_delay_arg = DeclareLaunchArgument(
        'nominal_actuator_delay_s',
        default_value='0.020',
        description='Nominal actuator transport delay assumption [s] (default: 0.020)'
    )
    nominal_actuator_slew_arg = DeclareLaunchArgument(
        'nominal_actuator_slew_rad_s',
        default_value='3.20',
        description='Nominal actuator slew rate limit [rad/s] (default: 3.20)'
    )
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='yaw_first_order',
        description='MPC prediction model type (kinematic or yaw_first_order)'
    )
    actuator_delay_s_arg = DeclareLaunchArgument(
        'actuator_delay_s',
        default_value='0.020',
        description='Actuator delay compensation in MPC dynamics [s] (default: 0.020)'
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
        steering_policy_arg,
        policy_hold_threshold_arg,
        policy_lattice_quantum_arg,
        nominal_actuator_delay_arg,
        nominal_actuator_slew_arg,
        model_type_arg,
        actuator_delay_s_arg,
        launch_sim_arg,
        autofocus_arg,
        OpaqueFunction(function=launch_setup)
    ])
