import cv2
import cv2.aruco as aruco
import numpy as np

#  Toggle this
SHOW_ORIENTATION = False   # True = show orientation, False = only 3D coords

# 🔹 Load calibration
camera_matrix = np.load("/home/scorpion/ratgun_ws/src/aruco_detection/aruco_detection/cam_caliberation/camera_matrix.npy")
dist_coeffs = np.load("/home/scorpion/ratgun_ws/src/aruco_detection/aruco_detection/cam_caliberation/dist_coeffs.npy")

marker_length = 0.06  # meters

cap = cv2.VideoCapture(0)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, params)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            x, y, z = tvecs[i][0]

            # 🔹 Always show 3D position
            text1 = f"ID:{ids[i][0]} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
            cv2.putText(frame, text1,
                        (10, 40 + i*60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            # 🔹 Draw axis
            cv2.drawFrameAxes(
                frame,
                camera_matrix,
                dist_coeffs,
                rvecs[i],
                tvecs[i],
                0.03
            )

            if SHOW_ORIENTATION:
                # 🔹 Compute orientation
                R, _ = cv2.Rodrigues(rvecs[i])

                sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
                singular = sy < 1e-6

                if not singular:
                    roll  = np.arctan2(R[2, 1], R[2, 2])
                    pitch = np.arctan2(-R[2, 0], sy)
                    yaw   = np.arctan2(R[1, 0], R[0, 0])
                else:
                    roll  = np.arctan2(-R[1, 2], R[1, 1])
                    pitch = np.arctan2(-R[2, 0], sy)
                    yaw   = 0

                roll_deg  = np.degrees(roll)
                pitch_deg = np.degrees(pitch)
                yaw_deg   = np.degrees(yaw)

                text2 = f"Yaw:{yaw_deg:.1f} Pitch:{pitch_deg:.1f} Roll:{roll_deg:.1f}"
                cv2.putText(frame, text2,
                            (10, 70 + i*60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 0), 2)

                print(f"ID {ids[i][0]} | X:{x:.3f} Y:{y:.3f} Z:{z:.3f} | "
                      f"Yaw:{yaw_deg:.1f} Pitch:{pitch_deg:.1f} Roll:{roll_deg:.1f}")
            else:
                print(f"ID {ids[i][0]} | X:{x:.3f} Y:{y:.3f} Z:{z:.3f}")

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()