from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    # Set paths to my package and the tb3 gazebo package
    pkg_tb3_nav_ctrl = get_package_share_directory('turtlebot_nav_control')
    pkg_tb3_gazebo = get_package_share_directory('gazebo_ros')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Set paths to the world we're launching and the robot urdf
    # world = os.path.join(pkg_tb3_nav_ctrl, 'worlds', 'empty.world')
    world = os.path.join(pkg_tb3_nav_ctrl, 'worlds', 'empty_world.world')
    robot_xacro = os.path.join(pkg_tb3_nav_ctrl, 'urdf', 'turtlebot_nav_control.urdf.xacro')

    # Define path to controller configuration file
    controller_config = os.path.join(pkg_tb3_nav_ctrl, 'config', 'controllers.yaml')

    # Define path to SLAM parameters file
    slam_params_file = os.path.join(pkg_tb3_nav_ctrl, 'config', 'mapper_params_online_async.yaml')

    # Process Xacro -> URDF
    robot_description = Command(['xacro ', robot_xacro])

    # Include Gazebo launch file (from turtlebot3_gazebo)
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_tb3_gazebo, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items(),
    # )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': 'false',  # ← KEY CHANGE: Disable GUI
            'server': 'true',
            'verbose': 'true'
        }.items(),
    )

    # Publish robot_description via robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn robot entity in gazebo (read robot_description topic)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot_nav_control', '-topic', 'robot_description'],
        output='screen',
        # remappings=[('/gazebo_ros_laser_controller/out', '/scan')]
    )

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner', # Spawner executable name is 'spawner'
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        name='joint_state_broadcaster_spawner',
        parameters=[controller_config],
        output='screen'
    )

    # Spawn diff_drive_controller
    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        name='diff_drive_controller_spawner',
        parameters=[controller_config],
        output='screen'
    )

    # SLAM Toolbox node
    slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[
        slam_params_file,
        {'use_sim_time': True}
    ],
    remappings=[
        ('/odom', '/diff_drive_controller/odom')
    ]
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
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Handler: Waits for the spawn_robot process to finish before starting SLAM
    load_slam_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot, 
            on_exit=[slam_node, simple_navigator], # <--- Launch SLAM and Navigator here
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller,
        rviz_node,
        load_slam_after_robot # Launches slam and simple_navigator
    ])

