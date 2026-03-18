from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('pile_inspection_controller'), 'launch', 'pile_inspection_controller.launch.py']
            )
        )
    )

    return LaunchDescription([
        controller_launch,
        Node(
            package='pile_inspection_controller',
            executable='dummy_tf_sim_node',
            name='dummy_tf_sim',
            output='screen',
            parameters=[
                {
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'poi_frame': 'poi',
                    'cmd_topic': 'cmd_vel',
                    'publish_rate_hz': 50.0,
                    'cmd_timeout_s': 0.5,
                    'poi_x': 2.0,
                    'poi_y': 3.0,
                    'poi_z': 0.0,
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_z': 0.0,
                    'initial_yaw_deg': 0.0,
                }
            ],
        ),
    ])
