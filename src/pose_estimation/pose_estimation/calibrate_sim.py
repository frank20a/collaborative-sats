import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import json, glob, os
from random import random


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

        # Create CVBridge instance
        self.bridge = CvBridge()

        # Create a client for the gazebo set_entity_state service
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        else:
            print('Service found')
        self.req = SetEntityState.Request()

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((cb[0]*cb[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:cb[0],0:cb[1]].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        
        while num_images > 0:
            if not self.flag: continue

            gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (cb[0], cb[1]), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                num_images -= 1
                print(num_images)

                # Draw and display the corners
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                cv.drawChessboardCorners(self.img, (cb[0], cb[1]), corners2, ret)
                self.flag = False
            cv.imshow('img', self.img)
            cv.waitKey(1)

    def send_request(self):

        self.req.state = EntityState()

        self.req.state.name = 'marker_cube'

        self.req.state.pose = Pose()
        self.req.state.pose.position.x = random() * 4 - 2
        self.req.state.pose.position.y = random() * 4 - 2
        self.req.state.pose.position.z = random() * 2

        self.req.state.twist = Twist()

        self.req.state.reference_frame = 'world'

        self.future = self.cli.call_async(self.req)

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.flag = True


def main(args=None):
    rclpy.init(args=args)
    viewer = Calibrator()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
