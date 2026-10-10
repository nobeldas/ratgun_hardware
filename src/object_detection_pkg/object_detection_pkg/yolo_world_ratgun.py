#!/usr/bin/env python3
"""Detect mice using YOLO-World and publish pixel coordinates."""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from ultralytics import YOLOWorld


class YoloWorldRatgun(Node):

    def __init__(self):
        super().__init__('yolo_world_ratgun')

        # Parameters
        self.declare_parameter(
            'image_topic',
            '/StereoNetNode/rectify_left_image'
        )

        self.declare_parameter(
            'output_topic',
            '/pixel_position'
        )

        self.declare_parameter(
            'model',
            'yolov8s-worldv2.pt'
        )

        self.declare_parameter(
            'confidence',
            0.005
        )

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_name = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value

        # Load YOLO-World
        self.model = YOLOWorld(model_name)

        self.model.set_classes([
            'keyboard'
        ])

        # Publisher
        self.mouse_publisher = self.create_publisher(
            PointStamped,
            output_topic,
            10
        )

        # Image subscriber
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # OpenCV window
        cv2.namedWindow(
            'YOLO-World Mouse Detection',
            cv2.WINDOW_NORMAL
        )

        self.get_logger().info(
            f'Listening for images on {image_topic}'
        )

        self.get_logger().info(
            f'Publishing mouse coordinates on {output_topic}'
        )

    def image_callback(self, msg):

        try:
            frame = self.image_to_bgr(msg)

        except ValueError as error:
            self.get_logger().error(str(error))
            return

        # YOLO inference
        result = self.model.predict(
            source=frame,
            conf=self.confidence,
            verbose=False
        )[0]

        best_detection = None
        best_confidence = -1.0

        # Draw every detected bounding box
        for box in result.boxes:

            x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()

            confidence = float(box.conf[0].cpu())

            center_u = (x1 + x2) / 2.0
            center_v = (y1 + y2) / 2.0

            # Bounding box
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )

            # Center point
            cv2.circle(
                frame,
                (int(center_u), int(center_v)),
                4,
                (0, 0, 255),
                -1
            )

            # Label
            label = (
                f'Mouse {confidence:.2f} '
                f'({center_u:.0f}, {center_v:.0f})'
            )

            cv2.putText(
                frame,
                label,
                (int(x1), max(int(y1) - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            # Select highest-confidence detection
            if confidence > best_confidence:

                best_confidence = confidence

                best_detection = (
                    center_u,
                    center_v
                )

        # Publish highest-confidence mouse
        if best_detection is not None:

            center_u, center_v = best_detection

            point_msg = PointStamped()

            # Preserve camera timestamp and frame
            point_msg.header = msg.header

            # Pixel coordinates
            point_msg.point.x = float(center_u)
            point_msg.point.y = float(center_v)

            # No depth available
            point_msg.point.z = 0.0

            self.mouse_publisher.publish(point_msg)

            self.get_logger().info(
                f'Mouse detected: '
                f'u={center_u:.1f}, '
                f'v={center_v:.1f}, '
                f'confidence={best_confidence:.3f}'
            )

        # Display annotated frame
        cv2.imshow(
            'YOLO-World Mouse Detection',
            frame
        )

        cv2.waitKey(1)

    @staticmethod
    def image_to_bgr(msg):
        """Convert ROS Image into OpenCV BGR image."""

        encoding = msg.encoding.lower()

        data = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        if encoding == 'nv12':

            expected_size = msg.step * msg.height * 3 // 2

            if data.size < expected_size:
                raise ValueError(
                    'NV12 image data is smaller than expected'
                )

            yuv = data[:expected_size].reshape(
                msg.height * 3 // 2,
                msg.step
            )[:, :msg.width]

            return cv2.cvtColor(
                yuv,
                cv2.COLOR_YUV2BGR_NV12
            )

        channels_by_encoding = {
            'bgr8': 3,
            'rgb8': 3,
            'bgra8': 4,
            'rgba8': 4,
            'mono8': 1
        }

        channels = channels_by_encoding.get(encoding)

        if channels is None:
            raise ValueError(
                f'Unsupported image encoding: {msg.encoding}'
            )

        expected_size = msg.step * msg.height

        if data.size < expected_size:
            raise ValueError(
                'Image data is smaller than expected'
            )

        row_width = msg.width * channels

        frame = data[:expected_size].reshape(
            msg.height,
            msg.step
        )[:, :row_width]

        frame = frame.reshape(
            msg.height,
            msg.width,
            channels
        )

        if encoding == 'bgr8':
            return frame.copy()

        if encoding == 'rgb8':
            return cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

        if encoding == 'bgra8':
            return cv2.cvtColor(
                frame,
                cv2.COLOR_BGRA2BGR
            )

        if encoding == 'rgba8':
            return cv2.cvtColor(
                frame,
                cv2.COLOR_RGBA2BGR
            )

        return cv2.cvtColor(
            frame,
            cv2.COLOR_GRAY2BGR
        )


def main(args=None):

    rclpy.init(args=args)

    node = YoloWorldRatgun()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()