import rclpy
from rclpy.node import Node

import numpy as np
import math

from tf2_ros import TransformBroadcaster

from tf_tree_pkg.matrix_to_tf import matrix_to_tf
from std_msgs.msg import Int32MultiArray


class DynamicTFNode(Node):

    def __init__(self):
        super().__init__('dynamic_tf_node')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            0.02,
            self.publish_tf
        )
        self.pan = 90  # Centered
        self.tilt = 90  # Centered

        self.declare_parameter('a1', 1.0)
        self.declare_parameter('a2', 1.0)
        self.declare_parameter('command_topic', 'pan_tilt_command')

        self.a1 = self.get_parameter('a1').value
        self.a2 = self.get_parameter('a2').value
        command_topic = self.get_parameter('command_topic').value

        self.subscription = self.create_subscription(
                Int32MultiArray,
                command_topic,
                self.command_callback,
                10)
        self.subscription  # prevent unused variable warning

    def command_callback(self, msg):
        self.pan, self.tilt = msg.data

    def publish_tf(self):

        stamp = self.get_clock().now().to_msg()
        pan = math.radians(self.pan - 90)
        tilt = math.radians(self.tilt - 90)

        # base_link -> servo1
        T_b_s1 = np.array([
            [math.cos(pan), -math.sin(pan), 0.0, 0.0],
            [math.sin(pan), math.cos(pan), 0.0, 0.0],
            [0.0, 0.0, 1.0, self.a1],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # servo1 --> servo2
        T_s1_s2 = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, self.a2],
            [0.0, 0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]) @ np.array([
            [math.cos(tilt), -math.sin(tilt), 0.0, 0.0],
            [math.sin(tilt), math.cos(tilt), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        tf_base_servo1 = matrix_to_tf(
            T_b_s1,
            'base_link',
            'servo1_link',
            stamp
        )

        tf_servo1_servo2 = matrix_to_tf(
            T_s1_s2,
            'servo1_link',
            'servo2_link',
            stamp
        )
        self.tf_broadcaster.sendTransform([
            tf_base_servo1,
            tf_servo1_servo2,
        ])


def main(args=None):

    rclpy.init(args=args)

    node = DynamicTFNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
