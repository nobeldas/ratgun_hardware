import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

id = 0

marker = aruco.generateImageMarker(aruco_dict, id=id, sidePixels=500)

cv2.imwrite(f"marker_id_{id}.png", marker)