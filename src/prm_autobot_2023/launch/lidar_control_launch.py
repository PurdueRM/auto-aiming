import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('prm_autobot_2023')
    prm_launch_dir = get_package_share_directory('prm_launch')
    launch_dir = os.path.join(bringup_dir, 'launch')

    
    control_comm_node = Node(package="control_communicator", executable="ControlCommunicatorNode")
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            )
        )
    )

    pointcloud_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pointcloud_to_laserscan.py'))
    )

    return LaunchDescription([
        control_comm_node,
        lidar_launch,
        pointcloud_to_laserscan_cmd
    ])