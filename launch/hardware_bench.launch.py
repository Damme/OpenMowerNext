"""Drive stack only: robot_state_publisher, ros2_control with the selected
hardware (OM_HARDWARE), joint_state_broadcaster, diff drive and blade
controllers. For bench tests (wheels up) and for testing the Worx plugin with
its firmware emulator:

    OM_HARDWARE=worx ros2 launch open_mower_next hardware_bench.launch.py worx_transport:=fake

Drive with TwistStamped on /diff_drive_base_controller/cmd_vel, blade with
Float64MultiArray [0..1] on /mower_controller/commands.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hardware_common import controller_parameters_file, robot_description  # noqa: E402


def launch_setup(context):
    share_directory = get_package_share_directory('open_mower_next')
    transport = LaunchConfiguration('worx_transport').perform(context)
    description = robot_description(share_directory, transport or None)

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', arguments=[name], output='screen')

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        # Jazzy's controller_manager takes the URDF from the ~/robot_description topic.
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controller_parameters_file(share_directory)],
             remappings=[('~/robot_description', '/robot_description')]),
        spawner('joint_state_broadcaster'),
        spawner('diff_drive_base_controller'),
        spawner('mower_controller'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('worx_transport', default_value='',
                              description='Worx only: spidev | fake (default: OM_WORX_TRANSPORT or spidev)'),
        OpaqueFunction(function=launch_setup),
    ])
