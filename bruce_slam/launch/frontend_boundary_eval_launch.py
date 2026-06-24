#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz",
    )
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="testing_data/dvl_fallback_0.2trans_0.4rot_fixed/",
        description="ROS bag path used for evaluation playback",
    )
    output_path_arg = DeclareLaunchArgument(
        "output_path",
        default_value="evaluation/recorded_raw_cfar.pkl",
        description="Pickle file for synced GT pose and raw CFAR points",
    )

    bruce_slam_dir = get_package_share_directory("bruce_slam")
    feature_config = os.path.join(bruce_slam_dir, "config", "feature.yaml")
    rviz_config = os.path.join(bruce_slam_dir, "rviz", "test_feature.rviz")

    frontend_eval_nodes = GroupAction(
        actions=[
            PushRosNamespace("bruce"),
            PushRosNamespace("slam"),
            Node(
                package="bruce_slam",
                executable="kalman_node.py",
                name="kalman",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="bruce_slam",
                executable="feature_extraction_node.py",
                name="feature_extraction",
                output="screen",
                parameters=[
                    feature_config,
                    {
                        "clustering.enable": False,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="bruce_slam",
                executable="eval_recorder_node.py",
                name="eval_recorder",
                output="screen",
                parameters=[
                    {
                        "output_path": LaunchConfiguration("output_path"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    bag_playback = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "--topics",
            "/dvl/data",
            "/sonar/ping",
            "/oceansim/robot/imu",
            "/oceansim/robot/gt_pose",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_arg,
            bag_path_arg,
            output_path_arg,
            frontend_eval_nodes,
            rviz_node,
            bag_playback,
        ]
    )
