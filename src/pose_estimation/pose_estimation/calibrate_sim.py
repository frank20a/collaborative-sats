import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import json, glob, os


class Calibrator(Node):
    def __init__(self):
        super().__init__("camera_calibrator")
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

    def calback(self, msg):
        # Read image and undistort
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main(args=None):
    rclpy.init(args=args)
    viewer = Calibrator()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
