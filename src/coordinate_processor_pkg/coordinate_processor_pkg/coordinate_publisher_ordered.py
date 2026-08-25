#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped

from cv_bridge import CvBridge
import cv2
import numpy as np

import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster


class ImageCloudRedDetector(Node):

    def __init__(self):
        super().__init__("image_cloud_red_detector")

        self.image_topic = "/StereoNetNode/rectify_left_image"
        self.cloud_topic = "/StereoNetNode/stereonet_pointcloud2"

        self.output_topic = "/red_object_center_image_cloud"
        self.child_frame = "target_tf"

        self.image_width = 640
        self.image_height = 352
        self.downsample_step = 2

        self.cloud_width = self.image_width // self.downsample_step
        self.cloud_height = self.image_height // self.downsample_step

        # 5x5 window in point cloud grid
        self.window_size = 5
        self.window_radius = self.window_size // 2

        self.latest_cloud = None

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            self.output_topic,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Image + PointCloud red detector started")

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def convert_image_to_bgr(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

        if msg.encoding == "nv12":
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)

        elif msg.encoding == "bgr8":
            pass

        elif msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return None

        return frame

    def detect_red_center(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 80])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 80])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 20:
            return None

        M = cv2.moments(largest)

        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return cx, cy

    def get_xyz_from_cloud(self, cloud_msg, cx, cy):
        cx_small = cx // self.downsample_step
        cy_small = cy // self.downsample_step

        if cx_small < 0 or cx_small >= self.cloud_width:
            return None

        if cy_small < 0 or cy_small >= self.cloud_height:
            return None

        points = list(pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=False
        ))

        sx = 0.0
        sy = 0.0
        sz = 0.0
        n = 0

        for dy in range(-self.window_radius, self.window_radius + 1):
            for dx in range(-self.window_radius, self.window_radius + 1):

                px = cx_small + dx
                py = cy_small + dy

                if px < 0 or px >= self.cloud_width:
                    continue

                if py < 0 or py >= self.cloud_height:
                    continue

                index = py * self.cloud_width + px

                if index < 0 or index >= len(points):
                    continue

                p = points[index]

                x = float(p[0])
                y = float(p[1])
                z = float(p[2])

                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue

                sx += x
                sy += y
                sz += z
                n += 1

        if n == 0:
            return None

        xc = float(sx / n)
        yc = float(sy / n)
        zc = float(sz / n)

        return xc, yc, zc

    def publish_point(self, cloud_msg, x, y, z):
        out = PointStamped()

        out.header.stamp = cloud_msg.header.stamp
        out.header.frame_id = cloud_msg.header.frame_id

        out.point.x = float(x)
        out.point.y = float(y)
        out.point.z = float(z)

        self.point_pub.publish(out)

    def publish_tf(self, cloud_msg, x, y, z):
        t = TransformStamped()

        t.header.stamp = cloud_msg.header.stamp
        t.header.frame_id = cloud_msg.header.frame_id
        t.child_frame_id = self.child_frame

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def image_callback(self, msg):
        if self.latest_cloud is None:
            self.get_logger().warn("No point cloud received yet")
            return

        frame = self.convert_image_to_bgr(msg)

        if frame is None:
            return

        center = self.detect_red_center(frame)

        if center is None:
            self.get_logger().warn("No red object found in image")
            return

        cx, cy = center

        xyz = self.get_xyz_from_cloud(self.latest_cloud, cx, cy)

        if xyz is None:
            self.get_logger().warn("Invalid 3D points around red center")
            return

        x, y, z = xyz

        self.publish_point(self.latest_cloud, x, y, z)
        self.publish_tf(self.latest_cloud, x, y, z)

        self.get_logger().info(
            f"Image center: cx={cx}, cy={cy} | "
            f"3D avg: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ImageCloudRedDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()