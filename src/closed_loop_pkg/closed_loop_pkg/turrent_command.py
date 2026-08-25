import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray


class TurretCommandNode(Node):

    def __init__(self):
        super().__init__('turret_command_node')

        self.declare_parameter('pan_tilt_topic', '/pan_tilt_commands')
        self.declare_parameter('fire_topic', '/fire_command')
        self.declare_parameter('turret_topic', '/turret_commands')

        pan_tilt_topic = self.get_parameter('pan_tilt_topic').value
        fire_topic = self.get_parameter('fire_topic').value
        turret_topic = self.get_parameter('turret_topic').value

        self.pan = None
        self.tilt = None
        self.fire = 0

        self.command_publisher = self.create_publisher(
            Int32MultiArray,
            turret_topic,
            10,
        )

        self.pan_tilt_subscription = self.create_subscription(
            Int32MultiArray,
            pan_tilt_topic,
            self.pan_tilt_callback,
            10,
        )

        self.fire_subscription = self.create_subscription(
            Int32,
            fire_topic,
            self.fire_callback,
            10,
        )

    def pan_tilt_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(
                'Invalid pan/tilt command. Expected [pan, tilt].'
            )
            return

        self.pan = int(msg.data[0])
        self.tilt = int(msg.data[1])
        self.publish_command()

    def fire_callback(self, msg):
        self.fire = int(msg.data)
        self.publish_command()

    def publish_command(self): # this is for the hardware to receive the command
        if self.pan is None or self.tilt is None:
            return

        command = Int32MultiArray()
        command.data = [self.pan, self.tilt, self.fire]
        self.command_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)

    node = TurretCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
