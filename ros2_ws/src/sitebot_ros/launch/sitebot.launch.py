#!/usr/bin/env python3
"""
SiteBot ROS2 Launch File

Launches:
1. MuJoCo simulation with ros2_control (mujoco_ros2_control)
2. Robot state publisher
3. Controllers (diff_drive, joint_state_broadcaster)
4. rosbridge for MCP access
5. Nav2 navigation stack (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_sitebot = get_package_share_directory('sitebot_ros')

    # Launch arguments
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Whether to launch Nav2 stack'
    )

    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Whether to launch rosbridge for MCP access'
    )

    # Robot description from xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_sitebot, 'urdf', 'sitebot.urdf.xacro'])
    ])
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_sitebot, 'config', 'sitebot_controllers.yaml'
    ])

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # MuJoCo ros2_control node
    # This runs MuJoCo simulation with ros2_control hardware interface
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config,
            {'mujoco_model_path': '/models/sitebot.xml'},
        ]
    )

    # Spawn controllers after MuJoCo starts
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    spawn_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # rosbridge for MCP access (WebSocket on port 9090)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rosbridge')),
        parameters=[{
            'port': 9090,
            'address': '',  # Listen on all interfaces
            'retry_startup_delay': 5.0,
        }]
    )

    # rosapi node for topic/service introspection via rosbridge
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rosbridge')),
    )

    # Nav2 (optional)
    nav2_params = PathJoinSubstitution([pkg_sitebot, 'config', 'nav2_params.yaml'])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    return LaunchDescription([
        # Arguments
        use_nav2_arg,
        use_rosbridge_arg,

        # Core nodes
        robot_state_pub,
        mujoco_node,

        # Controllers
        spawn_joint_state,
        spawn_diff_drive,

        # MCP access
        rosbridge_node,
        rosapi_node,

        # Navigation (optional)
        nav2_launch,
    ])
