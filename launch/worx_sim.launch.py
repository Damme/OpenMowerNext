"""Kinematic simulation of the Worx robot without Webots.

The Worx plugin runs with its firmware emulator (worx_transport:=fake), so
drive commands go through the real ros2_control/diff_drive/Worx code paths;
odometry is perfect and map->odom is a static transform at the start pose.
Brings up map_server (OM_MAP_PATH, OM_DATUM_LAT/LONG, 5 cm grid),
coverage_server, twist_mux and Nav2.

    OM_MAP_PATH=map.geojson OM_DATUM_LAT=.. OM_DATUM_LONG=.. \\
      ros2 launch open_mower_next worx_sim.launch.py start_x:=1.0 start_y:=2.0 start_yaw:=0.0
"""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hardware_common import controller_parameters_file, robot_description  # noqa: E402


def launch_setup(context):
    share = get_package_share_directory('open_mower_next')
    description = robot_description(share, 'fake')

    # diff drive publishes odom -> base_link here (no EKF in the kinematic sim).
    controllers = controller_parameters_file(share)
    with open(controllers, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    cfg['diff_drive_base_controller']['ros__parameters']['enable_odom_tf'] = True
    with open(controllers, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner', arguments=[name, *extra], output='screen')

    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ('start_x', 'start_y', 'start_yaw'))
    datum = [float(os.environ['OM_DATUM_LAT']), float(os.environ['OM_DATUM_LONG'])]

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers], remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        spawner('diff_drive_base_controller',
                '--controller-ros-args', '-r /diff_drive_base_controller/odom:=/odometry/filtered/map'),
        spawner('mower_controller'),
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--x', x, '--y', y, '--yaw', yaw, '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             parameters=[os.path.join(share, 'config', 'twist_mux.yaml')],
             remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]),
        Node(package='open_mower_next', executable='map_server_node', name='map_server', output='screen',
             parameters=[{'path': os.environ['OM_MAP_PATH'], 'datum': datum,
                          'grid.resolution': 0.05, 'grid.max_size': 4000}],
             remappings=[('map_grid', 'map_grid'), ('map', 'mowing_map')]),
        Node(package='open_mower_next', executable='coverage_server', output='screen'),
        Node(package='open_mower_next', executable='docking_helper', name='docking_helper', output='screen'),
        Node(package='open_mower_next', executable='worx_sim_dock.py', output='screen'),
        Node(package='open_mower_next', executable='mower_logic', output='screen',
             parameters=[{'require_gps': False,
                         'controller_id': LaunchConfiguration('pass_controller').perform(context),
                         'goal_checker_id': 'ftc_goal_checker' if LaunchConfiguration('pass_controller').perform(context) == 'FTC' else 'general_goal_checker',
                         'progress_checker_id': 'ftc_progress_checker' if LaunchConfiguration('pass_controller').perform(context) == 'FTC' else '',
                          'areas': LaunchConfiguration('areas').perform(context)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'nav2.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'autostart': 'true'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('OM_HARDWARE', 'worx'),
        SetEnvironmentVariable('OM_NAV2_PARAMS_OVERLAY', os.getenv(
            'OM_NAV2_PARAMS_OVERLAY',
            os.path.join(get_package_share_directory('open_mower_next'), 'config', 'hardware', 'worx_nav2.yaml'))),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('areas', default_value='', description='comma separated area ids (default all)'),
        DeclareLaunchArgument('pass_controller', default_value='FTC', description='FTC | FollowPath (RPP)'),
        OpaqueFunction(function=launch_setup),
    ])
