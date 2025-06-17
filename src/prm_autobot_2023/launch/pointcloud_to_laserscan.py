from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='pointcloud_to_laserscan launch file started successfully!'),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            remappings=[
                ('/cloud_in', '/livox/lidar'),
            ],
            parameters=[{
                'min_height': 0.05,
                'max_height': 0.7,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'range_min': 0.15,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1e-6
            }],
        )
    ])