import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package paths
    pkg_autonomousnavigator = get_package_share_directory('autonomousnavigator')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_frontier_exploration = get_package_share_directory('autonomousnavigator')  

    # Paths
    nav2_params_file = os.path.join(pkg_autonomousnavigator, 'config', 'nav2_params.yaml')

    # Start simulation
    start_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autonomousnavigator, 'launch', 'start_simulation.launch.py')
        )
    )

    # Start Nav2 with SLAM 
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True',       
            'autostart': 'True',
            'params_file': nav2_params_file,
        }.items()
    )

    # exploration
    explorer_node = Node(
        package='autonomousnavigator',
        executable='explorer',
        name='explorer',
        output='screen'
    )

    return LaunchDescription([
        start_simulation_launch,
        nav2_bringup,
        explorer_node
    ])
