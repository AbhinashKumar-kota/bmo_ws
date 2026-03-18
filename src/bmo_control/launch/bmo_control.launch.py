"""
BMO Control Launch File

Launches the BMO coordinator node with parameters from bmo_params.yaml.

Usage:
  ros2 launch bmo_control bmo_control.launch.py
  ros2 launch bmo_control bmo_control.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bmo_control = get_package_share_directory('bmo_control')
    params_file = os.path.join(pkg_bmo_control, 'config', 'bmo_params.yaml')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    bmo_coordinator = Node(
        package='bmo_control',
        executable='bmo_coordinator',
        name='bmo_coordinator',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        log_level_arg,
        bmo_coordinator,
    ])
