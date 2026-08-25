#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped

import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster


class RedPointDetector(Node):

    def __init__(self):
        super().__init__("red_point_detector")

        self.declare_parameter(
            "cloud_topic", "/StereoNetNode/stereonet_pointcloud2")
        self.declare_parameter("point_topic", "/red_object_center")
        self.declare_parameter("child_frame", "target_tf")
        self.declare_parameter("red_min", 120)
        self.declare_parameter("green_max", 80)
        self.declare_parameter("blue_max", 80)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.point_topic = self.get_parameter("point_topic").value
        self.child_frame = self.get_parameter("child_frame").value
        self.red_min = self.get_parameter("red_min").value
        self.green_max = self.get_parameter("green_max").value
        self.blue_max = self.get_parameter("blue_max").value

        for name, value in (
            ("red_min", self.red_min),
            ("green_max", self.green_max),
            ("blue_max", self.blue_max),
        ):
            if not 0 <= value <= 255:
                raise ValueError(
                    f'Parameter "{name}" must be between 0 and 255')

        self.sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            self.point_topic,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Red point detector started")

    def unpack_rgb(self, rgb_float):
        """
        PointCloud2 rgb field is packed inside FLOAT32.

        Bytes are usually:
        B, G, R, A
        """
        s = struct.pack("f", float(rgb_float))
        b, g, r, a = struct.unpack("BBBB", s)

        return r, g, b

    def is_red(self, r, g, b):
        """
        Return whether a point passes the red filter.

        Red point condition:
        R should be high.
        G and B should be low.
        """
        return (
            r > self.red_min
            and g < self.green_max
            and b < self.blue_max
        )

    def publish_red_point(self, msg, xc, yc, zc):
        out = PointStamped()

        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        out.point.x = float(xc)
        out.point.y = float(yc)
        out.point.z = float(zc)

        self.pub.publish(out)

    def publish_red_tf(self, msg, xc, yc, zc):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = self.child_frame

        t.transform.translation.x = float(xc)
        t.transform.translation.y = float(yc)
        t.transform.translation.z = float(zc)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def cloud_callback(self, msg):
        sx = 0.0
        sy = 0.0
        sz = 0.0
        n = 0

        points = pc2.read_points(
            msg,
            field_names=("x", "y", "z", "rgb"),
            skip_nans=True
        )

        for p in points:
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])
            rgb = p[3]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            r, g, b = self.unpack_rgb(rgb)

            if self.is_red(r, g, b):
                sx += x
                sy += y
                sz += z
                n += 1

        if n == 0:
            self.get_logger().warn("No red points found")
            return

        xc = float(sx / n)
        yc = float(sy / n)
        zc = float(sz / n)

        self.publish_red_point(msg, xc, yc, zc)
        self.publish_red_tf(msg, xc, yc, zc)

        self.get_logger().info(
            f"Red center: x={xc:.3f}, y={yc:.3f}, z={zc:.3f}, red_points={n}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = RedPointDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
