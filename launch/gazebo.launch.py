import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. Get the path to the gazebo_ros package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. Get the path to your custom package
    pkg_elderly_service_bot = get_package_share_directory('elderly_service_bot')

    # 3. Define the path to your world file
    world_file_path = os.path.join(pkg_elderly_service_bot, 'worlds', 'two_bhk_apart.world')

    # 4. Include the official Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            # THIS IS THE CRUCIAL FIX:
            'extra_gazebo_args': '--ros-args -p use_sim_time:=true'
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])
