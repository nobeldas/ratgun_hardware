#!/usr/bin/env python3

"""Convert AprilTag detection arrays to a target-detected heartbeat."""

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Bool


class AprilTagDetectionStatus(Node):
    """Publish whether the latest AprilTag frame contains a detection."""

    def __init__(self):
        super().__init__('apriltag_detection_status')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_detected_topic', '/target_detected')

        detections_topic = self.get_parameter('detections_topic').value
        target_detected_topic = self.get_parameter(
            'target_detected_topic').value

        self.publisher = self.create_publisher(
            Bool, target_detected_topic, 10)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self.detection_callback,
            10,
        )

    def detection_callback(self, msg):
        """Publish true when at least one AprilTag is present."""
        status = Bool()
        status.data = len(msg.detections) > 0
        self.publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
