import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

import cv2 as cv
import json, os
import numpy as np


def get_camera_calibration(filename = 'calibration.json'):
    with open(os.path.join(get_package_share_directory('pose_estimation'), filename), 'r') as f:
        cal = json.load(f)

    return np.asarray(cal['mtx']), np.asarray(cal['dist']), np.asarray(cal['new_mtx']), np.asarray(cal['roi'])


def undistort_crop(img, mtx, dist, new_mtx, roi):       
    dst = cv.undistort(img, mtx, dist, None, new_mtx)
    x, y, w, h = roi
    return dst[y:y+h, x:x+w]
    return dst


class UndistortedPublisher(Node):
    def __init__(self):
        super().__init__("undistort")

        self.bridge = CvBridge()
        self.declare_parameter('verbose', 0)
        self.declare_parameter('sim', False)
        self.declare_parameter('camera', '/front_cam')
        self.declare_parameter('fps', False)

        cam = ('/' + self.get_parameter('camera').get_parameter_value().string_value).replace('//', '/')
        self.fps_flag = self.get_parameter('fps').get_parameter_value().boolean_value

        self.publisher_disp = self.create_publisher(
            Image, 
            'undistorted', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        if self.get_parameter('sim').get_parameter_value().bool_value:
            self.get_logger().info("Starting to undistort from sim")
            self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration('sim_calibration.json')
            self.create_subscription(
                Image,
                (cam + '/image_raw').lstrip('/'),
                self.sim_callback,
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )
        else:
            self.get_logger().info("Starting to undistort from computer camera")
            self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration()
            self.cap = cv.VideoCapture(0)
            
            self.create_timer(1/60, self.normal_callback)

    def normal_callback(self):
        # self.get_logger().info("Normal Callback")
        _, img = self.cap.read()
        self.undistort(img)

    def sim_callback(self, msg):
        if self.verbose > 0:
            t = self.get_clock().now()
            
        # self.get_logger().info("Simulation Callback")
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.undistort(cv.cvtColor(img, cv.COLOR_RGB2BGR))
        
        if self.fps_flag:
            self.get_logger().info('Undistort duration: %.3f ms' % ((self.get_clock().now() - t).nanoseconds / 1e6))


    def undistort(self, img):
        dst = undistort_crop(img, self.mtx, self.dist, self.new_mtx, self.roi)

        if self.get_parameter('verbose').get_parameter_value().integer_value > 0:
            cv.imshow('distorted', img)
            cv.imshow('undistorted', dst)
            cv.waitKey(1)
        
        img_msg = self.bridge.cv2_to_imgmsg(dst)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_optical'
        self.publisher_disp.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    viewer = UndistortedPublisher()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()