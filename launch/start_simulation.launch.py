import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package paths
    pkg_Autonomousnavigator = get_package_share_directory('AutonomousNavigator')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Set Gazebo model path
    models_path = os.path.join(pkg_AutonomousNavigator, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        models_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    set_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=models_path
    )

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='3.5')
    y_pose = LaunchConfiguration('y_pose', default='4.8')
    world_file = LaunchConfiguration(
        'world',
        default=os.path.join(pkg_AutonomousNavigator, 'worlds', 'small_house.world')
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )
    urdf_xacro = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')
    robot_desc = xacro.process_file(urdf_xacro).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',  # run in global namespace
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': ''  # ensure no prefix is added
        }],
        output='screen'
    )
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')

    # Spawn Entity node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # RViz config file path
    rviz_config_file = os.path.join(pkg_elderly_service_bot, 'rviz', 'basic_turtlebot3.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Define the launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('x_pose', default_value='3.5', description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='4.8', description='Initial Y position'),

        set_model_path_env,
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity_node,
        rviz_node
    ])
