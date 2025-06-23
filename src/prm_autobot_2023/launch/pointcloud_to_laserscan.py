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
                'min_height': -0.5, # If upside down, negative means below the lidar
                'max_height': 0.01,
                'angle_min': -2.5,
                'angle_max': 2.5,
                'range_min': 0.275,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1e-6
            }],
        )
    ])