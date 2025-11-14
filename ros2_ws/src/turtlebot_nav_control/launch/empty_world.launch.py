from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Set paths to my package and the tb3 gazebo package
    pkg_tb3_nav_ctrl = get_package_share_directory('turtlebot_nav_control')
    pkg_tb3_gazebo = get_package_share_directory('gazebo_ros')

    # Set paths to the world we're launching and the robot urdf
    world = os.path.join(pkg_tb3_nav_ctrl, 'worlds', 'empty.world')
    robot_xacro = os.path.join(pkg_tb3_nav_ctrl, 'urdf', 'turtlebot_nav_control.urdf.xacro')

    # Define path to controller configuration file
    controller_config = os.path.join(pkg_tb3_nav_ctrl, 'config', 'controllers.yaml')

    # Process Xacro -> URDF
    robot_description = Command(['xacro ', robot_xacro])

    # Include Gazebo launch file (from turtlebot3_gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # # Spawn robot entity in gazebo (read robot_description topic)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot_nav_control', '-topic', 'robot_description'],
        output='screen'
    )

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner', # Spawner executable name is 'spawner'
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '-p', controller_config], # Pass config to spawner
        name='joint_state_broadcaster_spawner',  # Assign a name to the node for use in the dependency argument
        output='screen'
    )

    # Spawn diff_drive_controller
    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner', # Spawner executable name is 'spawner'
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '-p', controller_config], # Pass config to spawner
        output='screen'
    )

    # Publish robot_description via robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_tb3_nav_ctrl, 'rviz', 'turtlebot_nav_control.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Simple Navigator node
    simple_navigator = Node(
        package='turtlebot_nav_control',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller,
        robot_state_publisher,
        rviz_node,
        simple_navigator
    ])

