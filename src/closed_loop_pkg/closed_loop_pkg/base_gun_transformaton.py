import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float64MultiArray


class GunTFMatrixPublisher(Node):

    def __init__(self):
        super().__init__('gun_tf_matrix_publisher')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'gun_frame')
        self.declare_parameter('output_topic', '/gun_transform_matrix')

        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.publish_matrix
        )

    def publish_matrix(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            # Translation
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            # Quaternion
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Quaternion -> 3x3 rotation matrix
            R = Rotation.from_quat(
                [qx, qy, qz, qw]
            ).as_matrix()

            # Build 4x4 transformation matrix
            T = np.eye(4)

            T[:3, :3] = R
            T[:3, 3] = [x, y, z]

            # Publish
            msg = Float64MultiArray()
            msg.data = T.flatten().tolist()

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(str(e))


def main():
    rclpy.init()

    node = GunTFMatrixPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
