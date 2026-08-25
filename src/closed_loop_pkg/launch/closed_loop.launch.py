from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('closed_loop_pkg'),
        'config',
        'loop_config.yaml',
    ])

    return LaunchDescription([
        Node(
            package='closed_loop_pkg',
            executable='base_obj_coordinate',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='closed_loop_pkg',
            executable='base_gun_transformation',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='closed_loop_pkg',
            executable='pan_tilt_pub',
            parameters=[config_file],
            output='screen',
        ),
    ])
