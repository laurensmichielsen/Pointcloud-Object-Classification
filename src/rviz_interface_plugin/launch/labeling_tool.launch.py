from pathlib import Path
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import launch_ros
import os


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package="rviz_interface_plugin").find("rviz_interface_plugin")
    rviz_config = os.path.join(pkg_share, "rviz/rviz_labeling_config.rviz")

    return LaunchDescription(
        [
            Node(package="rosbag_frame_loader", executable="db3_frame_loader"),
            Node(package="labeling_assist", executable="labeling_assist"),
            Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", str(rviz_config)]),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.3",
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "labeling_pointcloud",
                ],
            ),
        ]
    )
