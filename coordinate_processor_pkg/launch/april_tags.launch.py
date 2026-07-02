from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_conf_file = PathJoinSubstitution([
        FindPackageShare('coordinate_processor_pkg'),
        'config',
        'april_tags.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_conf_file,  # actually the file is in coordinate_processor_pkg/config/ 
            description='give the config file path with the file name'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            parameters=[config_file],

            remappings=[
                ('image_rect', '/StereoNetNode/rectify_left_image'),
                ('camera_info', '/StereoNetNode/rectify_left_image/camera_info'),
            ],
            output='screen'

        )
    ])

# ros2 run apriltag_ros apriltag_node --ros-args \
#         --params-file april_tags.yaml \
#         -r image_rect:=/StereoNetNode/rectify_left_image \
#         -r /StereoNetNode/camera_info:=/StereoNetNode/rectify_left_image/camera_info