from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('tf_tree_pkg'),
        'config',
        'tf_tree.yaml',
    ])

    # Define the first node (e.g., a talker/publisher)
    dynamic_tf_node = Node(
        package='tf_tree_pkg',       # Name of the ROS2 package
        executable='dynamic_tf_node',            # Name of the target executable
        parameters=[config_file],
        output='screen'                 # Ensures logs print directly to terminal
    )

    # Define a second node (e.g., a listener/subscriber)
    static_tf_node = Node(
        package='tf_tree_pkg',
        executable='static_tf_node',
        parameters=[config_file],
        output='screen'
    )

    # Return the LaunchDescription object containing your nodes
    return LaunchDescription([
        dynamic_tf_node,
        static_tf_node
    ])
