import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_tb3_nav_ctrl = get_package_share_directory('turtlebot_nav_control')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_tb3_gazebo = get_package_share_directory('gazebo_ros')

    # Paths / Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(pkg_tb3_nav_ctrl, 'worlds', 'turtlebot3_world.world')
    robot_xacro = os.path.join(pkg_tb3_nav_ctrl, 'urdf', 'turtlebot_nav_control.urdf.xacro')

    nav2_params = os.path.join(pkg_tb3_nav_ctrl, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_tb3_nav_ctrl, 'maps', 'turtlebot3_world_map.yaml')

    rviz_config = os.path.join(pkg_tb3_nav_ctrl, 'rviz', 'turtlebot_nav_control.rviz')
    controller_config = os.path.join(pkg_tb3_nav_ctrl, 'config', 'controllers.yaml')

    # Process robot description
    robot_description = Command(['xacro ', robot_xacro])

    # Include Gazebo launch file (from turtlebot3_gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': 'false',
            'server': 'true',
            'verbose': 'true'
        }.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot', '-topic', 'robot_description'],
        output='screen',
    )

    # 
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config,
            {'use_sim_time': True},
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # This launches: Map Server, AMCL, Planner, Controller, BT Navigator, Lifecycle Manager
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'slam': 'True',
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'False',

        }.items()
    )

    # Rviz node with custom configuration file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Custom navigation node
    simple_navigator = Node(
        package='turtlebot_nav_control',
        executable='simple_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,

        ros2_control_node,
        joint_state_broadcaster,
        diff_drive_controller,

        nav2_bringup,

        rviz,
        simple_navigator,
    ])