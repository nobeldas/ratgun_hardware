from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('target_tf_pkg'),
        'config',
        'coordinate_processor.yaml',
    ])
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Coordinate processor parameter file',
        ),
        Node(
            package='target_tf_pkg',
            executable='coordinate_publisher_ordered',
            parameters=[config_file],
            output='screen',
        ),
    ])
