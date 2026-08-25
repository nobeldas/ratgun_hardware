from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    default_conf_file = PathJoinSubstitution([
        FindPackageShare('coordinate_processor_pkg'),
        'config',
        'april_tags.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_conf_file,
            description='give the config file path with the file name'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/StereoNetNode/rectify_left_image',
            description='Rectified image topic for AprilTag detection'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/StereoNetNode/rectify_left_image/camera_info',
            description='Camera info topic matching the rectified image'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[config_file],

            remappings=[
                ('image_rect', image_topic),
                ('/apriltag/image_rect', image_topic),
                ('camera_info', camera_info_topic),
                ('/camera_info', camera_info_topic),
                ('/StereoNetNode/camera_info', camera_info_topic),
            ],
            output='screen'

        )
    ])

# ros2 run apriltag_ros apriltag_node --ros-args \
#         --params-file april_tags.yaml \
#         -r image_rect:=/StereoNetNode/rectify_left_image \
#         -r /StereoNetNode/camera_info:=/StereoNetNode/rectify_left_image/camera_info
