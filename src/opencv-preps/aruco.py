import cv2 as cv
from undistort import *
from calibrate import getCalibration
import numpy as np

if __name__ == '__main__':
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_50)
    params = cv.aruco.DetectorParameters_create()
    cal = getCalibration()
    mtx = np.asarray(cal['mtx'])
    dist = np.asarray(cal['dist'])
    real_dim = 0.1335

    # Get calibration parameters
    mtx, dist, new_mtx, roi = get_camera_calibration()

    cam = cv.VideoCapture(0)
    while True:
        # Read image and undistort
        ret, img = cam.read()
        undistort_crop(img, mtx, dist, new_mtx, roi)

        # Detect markers
        corners, ids, rejected = cv.aruco.detectMarkers(img, dictionary, parameters=params)

        if len(corners) > 0:
            ids = ids.flatten()

            cv.aruco.drawDetectedMarkers(img, corners, ids)

            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, real_dim, mtx, dist)
            for rvec, tvec in zip(rvecs, tvecs):
                cv.aruco.drawAxis(img, mtx, dist, rvec, tvec, 0.025)


        cv.imshow('Image', img)
        cv.waitKey(1)