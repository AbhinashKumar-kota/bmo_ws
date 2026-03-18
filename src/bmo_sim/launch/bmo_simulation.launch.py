"""
BMO Simulation Launch File

Launches:
  1. Gazebo Harmonic with a world SDF (4 drones + lamp(s))
  2. ROS-Gazebo bridge (topics for all 4 drones)

Usage:
  ros2 launch bmo_sim bmo_simulation.launch.py
  ros2 launch bmo_sim bmo_simulation.launch.py world:=center_source_world.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_bmo_sim = get_package_share_directory('bmo_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World argument — default to single_source_world.sdf
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='single_source_world.sdf',
        description='World SDF filename (in bmo_sim/worlds/)'
    )

    # Path to the bridge config
    bridge_config = os.path.join(pkg_bmo_sim, 'config', 'bmo_bridge.yaml')

    # -----------------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH: tells Gazebo where to find our models
    # We need both:
    #   - Our models dir (crazyflie_bmo, tungsten_lamp)
    #   - Bitcraze's gazebo dir (original crazyflie model, used for mesh refs)
    # -----------------------------------------------------------------------
    models_path = os.path.join(pkg_bmo_sim, 'models')
    bitcraze_gazebo_path = os.path.join(
        os.path.expanduser('~'),
        'cf_ws', 'simulation_ws', 'crazyflie-simulation',
        'simulator_files', 'gazebo'
    )
    resource_path = models_path + ':' + bitcraze_gazebo_path

    # -----------------------------------------------------------------------
    # GZ_SIM_SYSTEM_PLUGIN_PATH: tells Gazebo where to find our plugin .so
    # -----------------------------------------------------------------------
    plugin_path = os.path.join(
        os.path.expanduser('~'),
        'bmo_ws', 'install', 'bmo_gz_plugins', 'lib'
    )

    # -----------------------------------------------------------------------
    # Launch Gazebo with our world
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [os.path.join(pkg_bmo_sim, 'worlds', ''),
                        LaunchConfiguration('world'), ' -r']
        }.items(),
    )

    # -----------------------------------------------------------------------
    # ROS-Gazebo bridge: bridges all drone topics to ROS 2
    # -----------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
        }],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        # Set environment variables BEFORE launching Gazebo
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        gz_sim,
        bridge,
    ])
