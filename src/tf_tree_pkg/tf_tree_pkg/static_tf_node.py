import rclpy
from rclpy.node import Node

import numpy as np

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tf_tree_pkg.matrix_to_tf import matrix_to_tf


class StaticTFNode(Node):

    def __init__(self):
        super().__init__('static_tf_node')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter('a3', 1.0)
        self.declare_parameter('tbc', [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, -1.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ])

        self.a3 = self.get_parameter('a3').value
        tbc = self.get_parameter('tbc').value

        if len(tbc) != 16:
            raise ValueError('Parameter "tbc" must contain exactly 16 values')

        # base_link -> lidar_link
        # servo2 -> gun_end
        T_s2_g = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, self.a3],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        # base_link -> camera_link
        T_b_c = np.array(tbc, dtype=float).reshape(4, 4)

        tf_s2_g = matrix_to_tf(
            T_s2_g,
            'servo2_link',
            'gun_end_link',
            self.get_clock().now().to_msg()
        )

        tf_b_c = matrix_to_tf(
                    T_b_c,
                    'base_link',
                    'camera_link',
                    self.get_clock().now().to_msg()
                )

        self.tf_broadcaster.sendTransform([tf_s2_g, tf_b_c])


def main(args=None):

    rclpy.init(args=args)

    node = StaticTFNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
