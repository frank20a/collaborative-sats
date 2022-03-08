import cv2 as cv
import numpy as np
import json, glob, os

def calibrateCameraLive(num_images: int = 15, cb: tuple = (8, 5)):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cb[0]*cb[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:cb[0],0:cb[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    cap = cv.VideoCapture(0)
    flag = True
    while num_images > 0:
        ret, img = cap.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (cb[0], cb[1]), None)

        # If found, add object points, image points (after refining them)
        if ret:
            if flag:
                objpoints.append(objp)
                imgpoints.append(corners)
                num_images -= 1
                print(num_images)

            # Draw and display the corners
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            cv.drawChessboardCorners(img, (cb[0], cb[1]), corners2, ret)
            flag = False
        else:
            flag = True
        cv.imshow('img', img)
        cv.waitKey(1)

    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    h, w = img.shape[:2]
    new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    with open('calibration.json', 'w') as f:
        json.dump({
            'mtx': np.asarray(mtx).tolist(),
            'dist': np.asarray(dist).tolist(),
            'rvecs': np.asarray(rvecs).tolist(),
            'tvecs': np.asarray(tvecs).tolist(),
            'new_mtx':np.asarray(new_mtx).tolist(),
            'roi': np.asarray(roi).tolist()
        }, f)
    return mtx, dist, rvecs, tvecs, new_mtx, roi


def calibrateCameraFiles(cb: tuple = (8, 5)):
     # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cb[0]*cb[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:cb[0],0:cb[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    
    images = glob.glob(os.path.join(os.getcwd(), '*.jpg'))

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (cb[0], cb[1]), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            cv.drawChessboardCorners(img, (cb[0], cb[1]), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(500)

    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    h, w = img.shape[:2]
    new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    with open('calibration.json', 'w') as f:
        json.dump({
            'mtx': np.asarray(mtx).tolist(),
            'dist': np.asarray(dist).tolist(),
            'rvecs': np.asarray(rvecs).tolist(),
            'tvecs': np.asarray(tvecs).tolist(),
            'new_mtx':np.asarray(new_mtx).tolist(),
            'roi': np.asarray(roi).tolist()
        }, f)
    return mtx, dist, rvecs, tvecs, new_mtx, roi


def getCalibration():
    with open('calibration.json', 'r') as f:
        return json.load(f)


if __name__ == '__main__':
    print(calibrateCameraLive())