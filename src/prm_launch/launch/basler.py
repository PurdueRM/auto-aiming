from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("prm_launch")
    basler_config = os.path.join(share_dir, "config", "basler.yaml")

    return LaunchDescription(
        [
            Node(
                package="pylon_ros2_camera_wrapper",
                executable="pylon_ros2_camera_wrapper",
                name="basler_camera",
                parameters=[basler_config],
                remappings=[
                    ("/basler_camera/image_raw", "/image_raw"),
                    ("/basler_camera/camera_info", "/camera_info"),
                ],
            ),
        ]
    )
