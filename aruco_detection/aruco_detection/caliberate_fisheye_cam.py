import cv2
import numpy as np
import glob


#for fisheye caliberation, we need more no of samples, like more than 15,
#  but in normal cam caliberation 10 also works well. 

#  Correct checkerboard size (from your image)
CHECKERBOARD = (9, 6)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

# prepare object points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)

objpoints = []  # 3D points
imgpoints = []  # 2D points

#  UPDATE THIS PATH if needed
images = glob.glob('/home/scorpion/ratgun_ws/src/aruco_detection/aruco_detection/aruco_caliberation_data/*')

print("Found images:", len(images))

image_size = None

for fname in images:
    img = cv2.imread(fname)

    if img is None:
        print(f" Failed to load {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if image_size is None:
        image_size = gray.shape[::-1]

    # find checkerboard
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners.reshape(1, -1, 2))

        # draw and show
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow('Corners', img)

        key = cv2.waitKey(200)

    else:
        print(f" Checkerboard not detected in {fname}")

cv2.destroyAllWindows()

print("Valid images used:", len(objpoints))

if len(objpoints) < 10:
    print(" Not enough valid images for calibration")
    exit()

#  Fisheye calibration
K = np.zeros((3, 3))
D = np.zeros((4, 1))

rvecs = []
tvecs = []

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    image_size,
    K,
    D,
    rvecs,
    tvecs,
    flags=(
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
        cv2.fisheye.CALIB_CHECK_COND +
        cv2.fisheye.CALIB_FIX_SKEW
    ),
    criteria=criteria
)

print("\n Calibration successful!")
print("\nCamera Matrix (K):\n", K)
print("\nDistortion Coefficients (D):\n", D)

#  Save results
np.save("K.npy", K)
np.save("D.npy", D)

#  Undistortion test
img = cv2.imread(images[0])
h, w = img.shape[:2]

map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, (w, h), cv2.CV_16SC2
)

undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

cv2.imshow("Original", img)
cv2.imshow("Undistorted", undistorted)
cv2.waitKey(0)
cv2.destroyAllWindows()