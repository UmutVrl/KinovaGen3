#########################################################################################
#    Kinova Gen3 Robotic Arm                                                            #
#    Camera Calibration (Part 2 - addition of undistortion)                             #
#    Taking a screenshot from the camera                                                #
#                                                                                       #
#    written by: U. Vural                                                               #
#    based on:                                                                          #
#    https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html                    #
#    https://github.com/niconielsen32/CameraCalibration/blob/main/calibration.py        #
#    see chessboard.png under resources folder                                          #
#                                                                                       #
#                                                                                       #
#    for KISS Project at Furtwangen University                                          #
#                                                                                       #
#########################################################################################
#    specs:                                                                             #
#    Python 3.9                                                                         #
#    Kinova Kortex 2.6.0                                                                #
#    Gen3 firmware Bundle 2.5.2-r.2                                                     #
#########################################################################################

import numpy as np
import cv2
import glob
import pickle
import pandas as pd
import time



################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

# Initialize the time tracker
first_capture_time = time.time()

chessboardSize = (10, 7)  # Be careful with the Board dimensions

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

size_of_chessboard_squares_mm = 20
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

#images = glob.glob('cameraCalibration/images/*.png')
images = glob.glob(r'resources\*.jpg')

for image in images:

    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
    print(ret)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1000)

cv2.destroyAllWindows()

############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
pickle.dump((cameraMatrix, dist), open("calibration.pkl", "wb"))
pickle.dump(cameraMatrix, open("cameraMatrix.pkl", "wb"))
pickle.dump(dist, open("dist.pkl", "wb"))

obj_to_read = pd.read_pickle(r'cameraMatrix.pkl')
obj_to_read2 = pd.read_pickle(r'dist.pkl')
obj_to_read3 = pd.read_pickle(r'calibration.pkl')
print("cam_matrix", obj_to_read)
print("dist", obj_to_read2)
print("calibration", obj_to_read3)
second_capture_time = time.time()
duration = second_capture_time - first_capture_time
days = int(duration // 86400)
hours = int(duration // 3600 % 24)
minutes = int(duration // 60 % 60)
seconds = int(duration % 60)
print(
    f" Program Run Duration: {hours} hours:{minutes} mins:{seconds} secs for {len(objpoints)} Object_pictures and {len(imgpoints)} Image_points")
print(duration)
print(len(objpoints))
print(len(imgpoints))

############## UNDISTORTION #####################################################

img = cv2.imread(r'resources\calib_screenshots3.jpg')
h, w = img.shape[:2]
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))

# Undistort
dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# crop the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]
cv2.imwrite('caliResult1.png', dst)

# Undistort with Remapping
mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w, h), 5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# crop the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]
cv2.imwrite('caliResult2.png', dst)

# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print("total error: {}".format(mean_error / len(objpoints)))

obj_to_read = pd.read_pickle(r'cameraMatrix.pkl')
obj_to_read2 = pd.read_pickle(r'dist.pkl')
obj_to_read3 = pd.read_pickle(r'calibration.pkl')
print("cam_matrix", obj_to_read)
print("dist", obj_to_read2)
print("calibration", obj_to_read3)
last_capture_time = time.time()
duration = last_capture_time - first_capture_time
days = int(duration // 86400)
hours = int(duration // 3600 % 24)
minutes = int(duration // 60 % 60)
seconds = int(duration % 60)
print(
    f" Program Run Duration: {hours} hours:{minutes} mins:{seconds} secs for {len(objpoints)} Object_pictures and {len(imgpoints)} Image_points")
