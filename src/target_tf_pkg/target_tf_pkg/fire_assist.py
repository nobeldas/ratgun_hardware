#!/usr/bin/env python3

"""Publish timed fire commands while a target is detected."""

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
    """Apply a fire duration and cooldown to target detections."""

    def __init__(self):
        super().__init__('fire_assist')

        self.declare_parameter('detection_topic', '/target_detected')
        self.declare_parameter('fire_topic', '/fire_command')

        detection_topic = self.get_parameter('detection_topic').value
        fire_topic = self.get_parameter('fire_topic').value

        self.fire_time = 0.10
        self.cool_down = 1.0

        self.target_detected = False
        self.firing = False
        self.fire_until = 0.0
        self.cool_down_until = 0.0

        self.fire_publisher = self.create_publisher(Int32, fire_topic, 10)
        self.detection_subscription = self.create_subscription(
            Bool,
            detection_topic,
            self.detection_callback,
            10,
        )
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.publish_fire(False)
        self.get_logger().info(
            f'Fire assist started SAFE: fire_time={self.fire_time:.2f}s, '
            f'cool_down={self.cool_down:.2f}s')

    def detection_callback(self, msg):
        """Record detection state and stop immediately on target loss."""
        self.target_detected = bool(msg.data)
        if not self.target_detected and self.firing:
            self.stop_firing(self.now_seconds())

    def now_seconds(self):
        """Return current node-clock time in seconds."""
        return self.get_clock().now().nanoseconds / 1.0e9

    def timer_callback(self):
        """Start and stop firing according to the two timing variables."""
        now = self.now_seconds()

        if self.firing:
            if not self.target_detected or now >= self.fire_until:
                self.stop_firing(now)
            return

        if self.target_detected and now >= self.cool_down_until:
            self.firing = True
            self.fire_until = now + self.fire_time
            self.publish_fire(True)

    def stop_firing(self, now):
        """Turn fire off and begin the cooldown."""
        self.firing = False
        self.cool_down_until = now + self.cool_down
        self.publish_fire(False)

    def publish_fire(self, detected):
        """Publish the fire-command value."""
        command = Int32()
        command.data = fire_value(detected)
        self.fire_publisher.publish(command)

    def force_safe(self):
        """Publish the safe fire-off command."""
        self.target_detected = False
        self.firing = False
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
