import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("target_prediction_pkg"),
        "config",
        "target_prediction.yaml",
    )

    return LaunchDescription([
        Node(
            package="target_prediction_pkg",
            executable="crlb_line_fitting",
            name="least_square_pred",
            output="screen",
            parameters=[config],
        )
    ])
