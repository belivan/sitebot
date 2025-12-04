#!/usr/bin/env python3
"""
SiteBot Gazebo Harmonic Launch File

Launches:
1. Gazebo Harmonic simulation with construction site world
2. ros_gz_bridge for Gazebo-ROS2 topic bridging
3. rosbridge for MCP access
4. Nav2 navigation stack (optional)
5. SLAM Toolbox (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_sitebot = get_package_share_directory('sitebot_ros')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Whether to launch Nav2 stack'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether to launch SLAM Toolbox'
    )

    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Whether to launch rosbridge for MCP access'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo in headless mode (no GUI)'
    )

    # Set Gazebo model path
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/gazebo/models:/gazebo/worlds'
    )

    # Disable GPU rendering for headless ARM64 container
    gz_render_engine = SetEnvironmentVariable(
        name='GZ_SIM_RENDER_ENGINE_SERVER_API_BACKEND',
        value='ogre'  # Use ogre instead of ogre2 for better compatibility
    )

    # Gazebo Harmonic simulation in server mode (headless)
    # -s = server only (no GUI), -r = run immediately
    # Note: GPU sensors removed for ARM64 Docker compatibility
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_ros_gz_sim, '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-s -r /gazebo/worlds/construction_site.sdf',
        }.items()
    )

    # Simulated Lidar publisher (CPU-based raycasting against known obstacles)
    # DISABLED: Now using real GPU lidar from Gazebo model
    # Keeping code for fallback if GPU lidar has issues
    # lidar_pub = Node(
    #     package='sitebot_ros',
    #     executable='lidar_publisher',
    #     name='simulated_lidar',
    #     output='screen',
    #     parameters=[{
    #         'num_beams': 360,
    #         'scan_rate': 10.0,
    #         'range_max': 30.0,
    #         'frame_id': 'laser_frame',
    #         'odom_topic': '/odom',  # Gazebo publishes odom here
    #     }]
    # )

    # ros_gz_bridge - bridges Gazebo topics to ROS2
    bridge_config = PathJoinSubstitution([pkg_sitebot, 'config', 'gz_bridge.yaml'])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    # TF static transform: odom -> base_link (from Gazebo diff_drive)
    # The diff_drive plugin publishes odom, we need to connect it to base_link
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'chassis']
    )

    # TF static transform: chassis -> laser_frame (legacy, for compatibility)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.45', '0', '0', '0', 'chassis', 'laser_frame']
    )

    # TF static transform: chassis -> sitebot/chassis/lidar (GPU lidar frame from Gazebo)
    static_tf_gpu_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_gpu_lidar',
        arguments=['0', '0', '0.42', '0', '0', '0', 'chassis', 'sitebot/chassis/lidar']
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

    # SLAM Toolbox (optional)
    slam_params = PathJoinSubstitution([pkg_sitebot, 'config', 'slam_params.yaml'])

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        condition=IfCondition(LaunchConfiguration('use_slam')),
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
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    return LaunchDescription([
        # Environment
        gz_model_path,
        gz_render_engine,

        # Arguments
        use_nav2_arg,
        use_slam_arg,
        use_rosbridge_arg,
        headless_arg,

        # Gazebo simulation
        gz_sim,

        # Bridges and transforms
        ros_gz_bridge,
        static_tf_odom,
        static_tf_laser,
        static_tf_gpu_lidar,

        # Sensors - GPU lidar now in Gazebo model (model.sdf)
        # lidar_pub,  # Disabled: using real Gazebo GPU lidar

        # MCP access
        rosbridge_node,
        rosapi_node,

        # SLAM (optional)
        slam_toolbox,

        # Navigation (optional)
        nav2_launch,
    ])
