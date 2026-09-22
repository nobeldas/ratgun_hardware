from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_projectile = LaunchConfiguration('use_projectile')
    config_file = PathJoinSubstitution([
        FindPackageShare('closed_loop_pkg'),
        'config',
        'loop_config.yaml',
    ])
    projectile_config_file = PathJoinSubstitution([
        FindPackageShare('projectile_pkg'),
        'config',
        'projectile_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_projectile',
            default_value='false',
            description='Use projectile compensation for pan/tilt commands.',
        ),
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
            condition=UnlessCondition(use_projectile),
        ),
        Node(
            package='projectile_pkg',
            executable='pan_tilt_projectile',
            parameters=[projectile_config_file],
            output='screen',
            condition=IfCondition(use_projectile),
        ),
        Node(
            package='closed_loop_pkg',
            executable='turrent_command',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='closed_loop_pkg',
            executable='base_servo2_transformation',
            parameters=[config_file],
            output='screen',
        )
    ])
