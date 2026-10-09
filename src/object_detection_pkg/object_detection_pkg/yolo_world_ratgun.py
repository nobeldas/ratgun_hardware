#!/usr/bin/env python3
"""Log YOLO-World rodent detections from a ROS image topic."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLOWorld


class YoloWorldRatgun(Node):
    """Detect small rodents and log their image-pixel coordinates."""

    def __init__(self):
        super().__init__('yolo_world_ratgun')

        self.declare_parameter(
            'image_topic',
            '/StereoNetNode/rectify_left_image',
        )
        self.declare_parameter('model', 'yolov8s-worldv2.pt')
        self.declare_parameter('confidence', 0.005)

        image_topic = self.get_parameter('image_topic').value
        model_name = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value

        self.model = YOLOWorld(model_name)
        self.model.set_classes(['small rodent'])

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Listening for images on {image_topic}'
        )

    def image_callback(self, msg):
        try:
            frame = self.image_to_bgr(msg)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        result = self.model.predict(
            source=frame,
            conf=self.confidence,
            verbose=False,
        )[0]

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
            confidence = float(box.conf[0].cpu())
            center_u = (x1 + x2) / 2.0
            center_v = (y1 + y2) / 2.0

            self.get_logger().info(
                'Mouse detected: '
                f'u={center_u:.1f}, v={center_v:.1f}, '
                f'confidence={confidence:.3f}'
            )

    @staticmethod
    def image_to_bgr(msg):
        """Convert common ROS Image encodings without using cv_bridge."""
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding == 'nv12':
            expected_size = msg.step * msg.height * 3 // 2
            if data.size < expected_size:
                raise ValueError(
                    'NV12 image data is smaller than height and step require'
                )
            yuv = data[:expected_size].reshape(
                msg.height * 3 // 2,
                msg.step,
            )[:, :msg.width]
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

        channels_by_encoding = {
            'bgr8': 3,
            'rgb8': 3,
            'bgra8': 4,
            'rgba8': 4,
            'mono8': 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')

        expected_size = msg.step * msg.height
        if data.size < expected_size:
            raise ValueError(
                'Image data is smaller than height and step require'
            )

        row_width = msg.width * channels
        frame = data[:expected_size].reshape(
            msg.height,
            msg.step,
        )[:, :row_width]
        frame = frame.reshape(msg.height, msg.width, channels)

        if encoding == 'bgr8':
            return frame
        if encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldRatgun()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
