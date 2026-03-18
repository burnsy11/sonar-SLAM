from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pile_inspection_controller',
            executable='pile_inspection_controller_node',
            name='pile_inspection_controller',
            output='screen',
            parameters=[
                {
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'poi_frame': 'poi',
                    'cmd_topic': 'cmd_vel',
                    'control_rate_hz': 20.0,
                    'x_target_to_poi': 0.4,
                    'y_target_to_poi': 0.0,
                    'initial_yaw_deg': 0.0,
                    'yaw_step_deg': 10.0,
                    'bottom_z_m': -7.0,
                    'top_z_m': 0.0,
                    'vertical_speed_mps': 0.5,
                    'kp_x': 1.0,
                    'kd_x': 0.2,
                    'kp_y': 1.0,
                    'kd_y': 0.2,
                    'kp_yaw': 1.2,
                    'kd_yaw': 0.25,
                    'kp_home_z': 1.0,
                    'max_vx': 0.8,
                    'max_vy': 0.8,
                    'max_vz': 0.6,
                    'max_wz': 1.2,
                    'xy_tolerance_m': 0.05,
                    'z_tolerance_m': 0.05,
                    'yaw_tolerance_deg': 2.0,
                }
            ],
        )
    ])
