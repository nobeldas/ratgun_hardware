from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the first node (e.g., a talker/publisher)
    dynamic_tf_node = Node(
        package='tf_tree_pkg',       # Name of the ROS2 package
        executable='dynamic_tf_node',            # Name of the target executable
        output='screen'                 # Ensures logs print directly to terminal
    )

    # Define a second node (e.g., a listener/subscriber)
    static_tf_node = Node(
        package='tf_tree_pkg',
        executable='static_tf_node',
        output='screen'
    )

    # Return the LaunchDescription object containing your nodes
    return LaunchDescription([
        dynamic_tf_node,
        static_tf_node
    ])
