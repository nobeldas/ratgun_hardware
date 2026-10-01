#!/usr/bin/env python3

"""Publish a fire command directly from target-detection state."""

import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Bool, Int32


def fire_value(detected):
    """Return the fire-command value for a detection state."""
    return 1 if detected else 0


class FireAssistNode(Node):
    """Turn the fire command on while a target is detected."""

    def __init__(self):
        super().__init__('fire_assist')

        self.declare_parameter('detection_topic', '/target_detected')
        self.declare_parameter('fire_topic', '/fire_command')

        detection_topic = self.get_parameter('detection_topic').value
        fire_topic = self.get_parameter('fire_topic').value

        self.fire_publisher = self.create_publisher(Int32, fire_topic, 10)
        self.detection_subscription = self.create_subscription(
            Bool,
            detection_topic,
            self.detection_callback,
            10,
        )

        self.publish_fire(False)
        self.get_logger().info('Fire assist started SAFE')

    def detection_callback(self, msg):
        """Publish fire on for detection and off for no detection."""
        self.publish_fire(msg.data)

    def publish_fire(self, detected):
        """Publish the fire-command value."""
        command = Int32()
        command.data = fire_value(detected)
        self.fire_publisher.publish(command)

    def force_safe(self):
        """Publish the safe fire-off command."""
        self.publish_fire(False)


def main(args=None):
    shutdown_requested = threading.Event()
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = FireAssistNode()

    def request_safe_shutdown(_signum, _frame):
        shutdown_requested.set()

    signal.signal(signal.SIGINT, request_safe_shutdown)
    signal.signal(signal.SIGTERM, request_safe_shutdown)

    try:
        while rclpy.ok() and not shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if rclpy.ok():
            node.force_safe()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
