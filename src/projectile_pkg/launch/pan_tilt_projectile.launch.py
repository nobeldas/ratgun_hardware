from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('projectile_pkg'),
        'config',
        'projectile_config.yaml',
    ])

    return LaunchDescription([
        Node(
            package='projectile_pkg',
            executable='pan_tilt_projectile',
            parameters=[config_file],
            output='screen',
        )
    ])
