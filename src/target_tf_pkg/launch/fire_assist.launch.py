from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('target_tf_pkg'),
        'config',
        'fire_assist.yaml',
    ])

    return LaunchDescription([
        Node(
            package='target_tf_pkg',
            executable='fire_assist',
            parameters=[config_file],
            output='screen',
        ),
    ])
