import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

import cv2 as cv
import json, os
import numpy as np

bridge = CvBridge()


def get_camera_calibration():
    with open(os.path.join(get_package_share_directory('pose_estimation'), 'calibration.json'), 'r') as f:
        cal = json.load(f)

    return np.asarray(cal['mtx']), np.asarray(cal['dist']), np.asarray(cal['new_mtx']), np.asarray(cal['roi'])


def undistort_crop(img, mtx, dist, new_mtx, roi):       
    dst = cv.undistort(img, mtx, dist, None, new_mtx)
    x, y, w, h = roi
    # return dst[y:y+h, x:x+w]
    return dst


class UndistortedPublisher(Node):
    def __init__(self):
        super().__init__("camera_calibrator")

        self.declare_parameter('verbose', 0)
        
        
        self.publisher_disp = self.create_publisher(
            Image, 
            '/undistorted', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration()
        self.cap = cv.VideoCapture(0)
        
        self.create_timer(1/60, self.callback)

    def callback(self):
        _, img = self.cap.read()
        dst = undistort_crop(img, self.mtx, self.dist, self.new_mtx, self.roi)

        if self.get_parameter('verbose').get_parameter_value().integer_value > 0:
            cv.imshow('distorted', img)
            cv.imshow('undistorted', dst)
            cv.waitKey(1)

        self.publisher_disp.publish(bridge.cv2_to_imgmsg(dst))


def main(args=None):
    rclpy.init(args=args)
    viewer = UndistortedPublisher()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()