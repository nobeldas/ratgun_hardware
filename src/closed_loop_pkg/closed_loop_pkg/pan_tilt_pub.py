import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray


class PanTiltSubscriber(Node):

    def __init__(self):
        super().__init__('pan_tilt_node')

        self.target_xyz = None
        self.gun_transform = None

        self.declare_parameter('target_topic', '/target_tf_position')
        self.declare_parameter('output_topic', '/gun_transform_matrix')
        self.declare_parameter('loop_topic', 'pan_tilt_command')
        self.declare_parameter('a1', 1.0)
        self.declare_parameter('a2', 1.0)
        self.declare_parameter('a3', 1.0)

        self.target_topic = self.get_parameter(
                    'target_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
                            'output_topic').get_parameter_value().string_value
        self.loop_topic = self.get_parameter(
                            'loop_topic').get_parameter_value().string_value
        self.a1 = self.get_parameter('a1').value
        self.a2 = self.get_parameter('a2').value
        self.a3 = self.get_parameter('a3').value

        self.create_subscription(
            Point,
            self.target_topic,
            self.target_callback,
            10)

        self.create_subscription(
            Float64MultiArray,
            self.output_topic,
            self.output_topic_callback,
            10)
        
        self.pub_commands = self.create_publisher(
            Int32MultiArray,
            self.loop_topic,
            10)
        
        self.create_timer(
            0.02,
            self.publish_commands
        )
    def publish_commands(self):
        if self.target_xyz is None or self.gun_transform is None:
            return

#       gun_rotation = self.gun_transform[:3, :3]
        gun_position = self.gun_transform[:3, 3]

        target_from_gun_base = self.target_xyz - gun_position
        pan = np.arctan2(target_from_gun_base[1], target_from_gun_base[0])

        x = target_from_gun_base[0]
        y = target_from_gun_base[1]
        z = target_from_gun_base[2]

        r = x * np.cos(pan) + y * np.sin(pan)
        h = z - self.a1 - self.a2

        R = np.sqrt(r*r + h*h)

        alpha = np.arctan2(r, h)

        tilt1 = np.arccos(self.a3 / R) - alpha
        tilt2 = -np.arccos(self.a3 / R) - alpha

        msg = Int32MultiArray()
        msg.data = [int(np.degrees(pan)), int(np.degrees(tilt1))]
        self.pub_commands.publish(msg)



    def output_topic_callback(self, msg):
        if len(msg.data) != 16:
            self.get_logger().error(
              f'Expected 16 matrix values, received {len(msg.data)}'
          )
            return

        self.gun_transform = np.asarray(
            msg.data,
            dtype=float
        ).reshape(4, 4)

    def target_callback(self, msg):
        self.target_xyz = np.array(
            [msg.x, msg.y, msg.z],
            dtype=float
       )


def main(args=None):
    rclpy.init(args=args)

    pan_tilt_subscriber = PanTiltSubscriber()

    rclpy.spin(pan_tilt_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pan_tilt_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()