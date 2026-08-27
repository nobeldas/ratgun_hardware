import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point


class TFPositionPublisher(Node):

    def __init__(self):
        super().__init__('tf_position_publisher')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'target_tf')
        self.declare_parameter('target_topic', '/target_tf_position')

        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value
        self.target_topic = self.get_parameter(
            'target_topic').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            Point,
            self.target_topic,
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.publish_position
        )

    def publish_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            msg = Point()

            msg.x = tf.transform.translation.x
            msg.y = tf.transform.translation.y
            msg.z = tf.transform.translation.z

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(str(e))


def main():
    rclpy.init()

    node = TFPositionPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
