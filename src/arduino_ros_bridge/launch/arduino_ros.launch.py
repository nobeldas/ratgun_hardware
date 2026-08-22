from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_name = LaunchConfiguration('port_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Serial port connected to the Arduino'
        ),

        Node(
            package='arduino_ros_bridge',
            executable='arduino_ros_bridge',
            parameters=[{
                'port_name': port_name
            }]
        ),
        
        Node(
            package='arduino_ros_bridge',
            executable='control_led_enter'
        )
    ])
