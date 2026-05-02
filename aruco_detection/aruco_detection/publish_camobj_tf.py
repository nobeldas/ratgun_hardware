import rclpy
from rclpy.node import Node

import cv2
import cv2.aruco as aruco
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from scipy.spatial.transform import Rotation as R


class ArucoTFNode(Node):

    def __init__(self):
        super().__init__('aruco_tf_node')

        # TF broadcasters
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # 🔹 Publish static camera frame
        self.publish_static_camera()

        # 🔹 Load calibration
        self.camera_matrix = np.load(
            "/home/scorpion/ratgun_ws/src/aruco_detection/aruco_detection/cam_caliberation/camera_matrix.npy"
        )
        self.dist_coeffs = np.load(
            "/home/scorpion/ratgun_ws/src/aruco_detection/aruco_detection/cam_caliberation/dist_coeffs.npy"
        )

        self.marker_length = 0.06

        # OpenCV
        self.cap = cv2.VideoCapture(0)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.params)

        # Timer loop (~30 Hz)
        self.timer = self.create_timer(0.03, self.process_frame)

    def publish_static_camera(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "camera"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 2.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_br.sendTransform(t)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            for i in range(len(ids)):
                marker_id = ids[i][0]

                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                # Convert rotation vector → quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_mat).as_quat()

                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera"
                t.child_frame_id = f"aruco_{marker_id}"

                # Position (camera frame)
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])

                # Orientation
                t.transform.rotation.x = float(quat[0])
                t.transform.rotation.y = float(quat[1])
                t.transform.rotation.z = float(quat[2])
                t.transform.rotation.w = float(quat[3])

                self.br.sendTransform(t)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()