from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


import os

from ament_index_python.packages import get_package_share_path

import os

def generate_launch_description():
    share_dir = get_package_share_directory("prm_launch")
    mv_config = os.path.join(share_dir, "config", "mv_red_bidc.yaml")

    opencv_armor_detector_config = os.path.join(share_dir, "config", "opencv_armor_detector_red_bidc.yaml")

    return LaunchDescription(
        [
            Node(package="mv_publisher", executable="MVCameraNode", parameters=[mv_config]),
            Node(
            package='opencv_armor_detector',
            executable='OpenCVArmorDetectorNode',  
        ),
        Node(
            package='pose_estimator',
            executable='PoseEstimatorNode',
        ),
    ])
