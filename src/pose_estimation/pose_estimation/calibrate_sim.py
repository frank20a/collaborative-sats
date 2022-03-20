import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import json, glob, os


class Calibrator(Node):
    def __init__(self):
        super().__init__("sim_camera_calibrator")
        self.declare_parameter('num-images', 15)
        self.declare_parameter('chessboard-h', 7)
        self.declare_parameter('chessboard-w', 7)

        self.create_subscription(
            Image,
            '/camera_raw/image_raw',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        cb = (
            self.get_parameter('chessboard-h').get_parameter_value().integer_value,
            self.get_parameter('chessboard-w').get_parameter_value().integer_value
        )
        self.num_images = self.get_parameter('num-images').get_parameter_value().integer_value

        # termination criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((cb[0]*cb[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:cb[0],0:cb[1]].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

        self.trigger = False
        self.finished = False

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        for i in range(self.num_images):
            pass


    def callback(self, msg):
        # Read image and undistort
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(self.gray, (self.cb[0], self.cb[1]), None)
        
        if ret:
            if self.trigger:
                self.trigger = False

                self.objpoints.append(self.objp)
                self.imgpoints.append(corners)
                self.num_images -= 1
                print(self.num_images)

            # Draw and display the corners
            corners2 = cv.cornerSubPix(self.gray, corners, (11,11), (-1,-1), self.criteria)
            cv.drawChessboardCorners(self.img, (self.cb[0], self.cb[1]), corners2, ret)
        
        cv.imshow('img', self.img)
        cv.waitKey(1)

        if self.num_images <= 0:
            self.finish()
            return

    def finish(self):
        cv.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)
        h, w = self.img.shape[:2]
        new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        with open(os.path.join(get_package_share_directory('pose_estimation'), 'sim_calibration.json'), 'w') as f:
            json.dump({
                'mtx': np.asarray(mtx).tolist(),
                'dist': np.asarray(dist).tolist(),
                'rvecs': np.asarray(rvecs).tolist(),
                'tvecs': np.asarray(tvecs).tolist(),
                'new_mtx':np.asarray(new_mtx).tolist(),
                'roi': np.asarray(roi).tolist()
            }, f)

        # return mtx, dist, rvecs, tvecs, new_mtx, roi
        self.destroy_node()



def main(args=None):
    rclpy.init(args=args)
    viewer = Calibrator()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
