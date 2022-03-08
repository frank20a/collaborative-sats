import cv2 as cv
import json
import numpy as np

def get_camera_calibration():
    with open('calibration.json', 'r') as f:
        cal = json.load(f)

    return np.asarray(cal['mtx']), np.asarray(cal['dist']), np.asarray(cal['new_mtx']), np.asarray(cal['roi'])

def undistort_crop(img, mtx, dist, new_mtx, roi):       
    dst = cv.undistort(img, mtx, dist, None, new_mtx)
    x, y, w, h = roi
    return dst[y:y+h, x:x+w]

if __name__ == '__main__':
    mtx, dist, new_mtx, roi = get_camera_calibration()

    cap = cv.VideoCapture(0)
    while True:
        ret, img = cap.read()
        cv.imshow('distorted', img)
        cv.imshow('undistorted', undistort_crop(img, mtx, dist, new_mtx, roi))
        cv.waitKey(1)