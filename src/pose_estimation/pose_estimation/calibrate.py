import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np
import json, glob, os


def calibrateCameraLive(num_images: int = 15, cb: tuple = (8, 5), camera_name = 'ip'):
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

    with open(os.path.join(get_package_share_directory('pose_estimation'), camera_name + '_calibration.json'), 'w') as f:
        json.dump({
            'mtx': np.asarray(mtx).tolist(),
            'dist': np.asarray(dist).tolist(),
            'rvecs': np.asarray(rvecs).tolist(),
            'tvecs': np.asarray(tvecs).tolist(),
            'new_mtx':np.asarray(new_mtx).tolist(),
            'roi': np.asarray(roi).tolist(),
            'h': h,
            'w': w
        }, f)
    return mtx, dist, rvecs, tvecs, new_mtx, roi


def calibrateCameraFiles(cb: tuple = (8, 5), folder = os.getcwd()):
     # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cb[0]*cb[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:cb[0],0:cb[1]].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    
    images = glob.glob(os.path.join(folder, '*.jpg'))

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

    with open(os.path.join(get_package_share_directory('pose_estimation'), 'calibration.json'), 'w') as f:
        json.dump({
            'mtx': np.asarray(mtx).tolist(),
            'dist': np.asarray(dist).tolist(),
            'rvecs': np.asarray(rvecs).tolist(),
            'tvecs': np.asarray(tvecs).tolist(),
            'new_mtx':np.asarray(new_mtx).tolist(),
            'roi': np.asarray(roi).tolist(),
            'h': h,
            'w': w
        }, f)
    return mtx, dist, rvecs, tvecs, new_mtx, roi


def get_camera_calibration():
    with open(os.path.join(get_package_share_directory('pose_estimation'), 'calibration.json'), 'r') as f:
        cal = json.load(f)

    return np.asarray(cal['mtx']), np.asarray(cal['dist']), np.asarray(cal['new_mtx']), np.asarray(cal['roi']), cal['h'], cal['w']


class Calibrator(Node):
    def __init__(self):
        super().__init__("camera_calibrator")
        self.declare_parameter('num_images', 15)
        self.declare_parameter('chessboard_h', 8)
        self.declare_parameter('chessboard_w', 5)
        self.declare_parameter('type', 'live')
        self.declare_parameter('folder', '/home/frank20a/calibration_imgs')
        self.declare_parameter('camera_name', 'ip')


        if self.get_parameter('type').get_parameter_value().string_value == 'live':
            calibrateCameraLive(
                self.get_parameter('num_images').get_parameter_value().integer_value,
                (
                    self.get_parameter('chessboard_h').get_parameter_value().integer_value,
                    self.get_parameter('chessboard_w').get_parameter_value().integer_value,
                ),
                self.get_parameter('camera_name').get_parameter_value().string_value
            )
            self.destroy_node()

        elif self.get_parameter('type').get_parameter_value().string_value == 'file':
            calibrateCameraFiles(
                (
                    self.get_parameter('chessboard_h').get_parameter_value().integer_value,
                    self.get_parameter('chessboard_w').get_parameter_value().integer_value
                ),
                self.get_parameter('folder').get_parameter_value().string_value
            )
            self.destroy_node()

        if self.get_parameter('type').get_parameter_value().string_value == 'topic':

            self.bridge = CvBridge()

            self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
            self.cb = (
                    self.get_parameter('chessboard_h').get_parameter_value().integer_value,
                    self.get_parameter('chessboard_w').get_parameter_value().integer_value,
                )
            self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

            # termination criteria
            self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            self.objp = np.zeros((self.cb[0]*self.cb[1], 3), np.float32)
            self.objp[:,:2] = np.mgrid[0:self.cb[0],0:self.cb[1]].T.reshape(-1,2)

            # Arrays to store object points and image points from all the images.
            self.objpoints = [] # 3d point in real world space
            self.imgpoints = [] # 2d points in image plane.

            self.flag = True
            self.create_subscription(Image, 'camera/image_raw', self.callback, QoSPresetProfiles.get_from_short_key('sensor_data'))

        else:
            print('Invalid parameter "type"')

        
    def callback(self, msg):

        if self.num_images > 0:
            self.img = self.bridge.imgmsg_to_cv2(msg)
            self.gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(self.gray, (self.cb[0], self.cb[1]), None)

            # If found, add object points, image points (after refining them)
            if ret:
                if self.flag:
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners)
                    self.num_images -= 1
                    print(self.num_images)

                # Draw and display the corners
                corners2 = cv.cornerSubPix(self.gray,corners, (11,11), (-1,-1), self.criteria)
                cv.drawChessboardCorners(self.img, (self.cb[0], self.cb[1]), corners2, ret)
                self.flag = False
            else:
                self.flag = True
            cv.imshow('img', self.img)
            cv.waitKey(1)

        else:

            cv.destroyAllWindows()
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)
            h, w = self.img.shape[:2]
            new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

            with open(os.path.join(get_package_share_directory('pose_estimation'), self.camera_name + '_calibration.json'), 'w') as f:
                json.dump({
                    'mtx': np.asarray(mtx).tolist(),
                    'dist': np.asarray(dist).tolist(),
                    'rvecs': np.asarray(rvecs).tolist(),
                    'tvecs': np.asarray(tvecs).tolist(),
                    'new_mtx':np.asarray(new_mtx).tolist(),
                    'roi': np.asarray(roi).tolist(),
                    'h': h,
                    'w': w
                }, f)
            self.get_logger().info("DONE AND SAVED")
            while True:
                pass
        
def main(args=None):
    rclpy.init(args=args)
    viewer = Calibrator()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
