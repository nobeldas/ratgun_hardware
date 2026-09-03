from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('cctag_tf_pkg'),
        'config',
        'cctag_3rings.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='CCTag detector parameter file',
        ),
        Node(
            package='cctag_tf_pkg',
            executable='cctag_tf_node',
            name='cctag_tf_node',
            parameters=[config_file],
            output='screen',
        ),
    ])
