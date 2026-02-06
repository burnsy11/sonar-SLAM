#!/usr/bin/env python3
"""
ROS2 Launch file for BRUCE SLAM system
Converted from ROS1 slam.launch
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM features'
    )
    
    kalman_dead_reckoning_arg = DeclareLaunchArgument(
        'kalman_dead_reckoning',
        default_value='true',
        description='Use Kalman filter for dead reckoning'
    )
    
    # Offline mode arguments
    file_arg = DeclareLaunchArgument(
        'file',
        default_value='',
        description='ROS bag file for offline mode'
    )
    
    start_arg = DeclareLaunchArgument(
        'start',
        default_value='0.0',
        description='Start time for bag playback'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='-1',
        description='Duration for bag playback'
    )
    
    kill_arg = DeclareLaunchArgument(
        'kill',
        default_value='false',
        description='Kill on node exit'
    )
    
    # Get package directories
    bruce_slam_dir = get_package_share_directory('bruce_slam')
    
    # Config file paths
    dead_reckoning_config = os.path.join(bruce_slam_dir, 'config', 'dead_reckoning.yaml')
    feature_config = os.path.join(bruce_slam_dir, 'config', 'feature.yaml')
    
    slam_config = os.path.join(bruce_slam_dir, 'config', 'slam.yaml')
    
    # RViz config - use ROS2 compatible version
    rviz_config = os.path.join(bruce_slam_dir, 'rviz', 'video.rviz')
    
    # Online mode nodes (when file argument is empty)
    online_nodes = GroupAction(
        condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('file'), "' != ''"])),

        actions=[
            PushRosNamespace('bruce'),
            PushRosNamespace('slam'),
            
            
            
            # Dead reckoning node (if not using Kalman)
            Node(
                condition=UnlessCondition(LaunchConfiguration('kalman_dead_reckoning')),
                package='bruce_slam',
                executable='dead_reckoning_node.py',
                name='dead_reckoning',
                output='screen',
                parameters=[dead_reckoning_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            
            # Kalman filter node (if using Kalman)
            Node(
                condition=IfCondition(LaunchConfiguration('kalman_dead_reckoning')),
                package='bruce_slam',
                executable='kalman_node.py',
                name='kalman',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            
            # Feature extraction node
            Node(
                package='bruce_slam',
                executable='feature_extraction_node.py',
                name='feature_extraction',
                output='screen',
                parameters=[feature_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            
            # SLAM node
            Node(
                package='bruce_slam',
                executable='slam_node.py',
                name='slam',
                output='screen',
                parameters=[
                    slam_config,
                    {'enable_slam': LaunchConfiguration('enable_slam')},
                    {'save_fig': False},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            ),
        ]
    )
    
    # Static transform publisher (map to world)
    # In ROS2, we use a Node instead of a separate executable
    map_to_world_tf = Node(
        condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('file'), "' != ''"])),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0.0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Offline mode would require additional handling
    # For now, focusing on online mode as offline mode needs rosbag2 API
    
    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        enable_slam_arg,
        kalman_dead_reckoning_arg,
        file_arg,
        start_arg,
        duration_arg,
        kill_arg,
        online_nodes,
        map_to_world_tf,
        rviz_node,
    ])
