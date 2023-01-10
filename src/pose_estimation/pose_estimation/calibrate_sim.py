import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler

import cv2 as cv
import numpy as np
import json, glob, os
from random import random
from numpy import pi


class Calibrator(Node):
    def __init__(self):
        super().__init__("sim_camera_calibrator")
        self.declare_parameter('num_images', 100)
        self.declare_parameter('chessboard_h', 7)
        self.declare_parameter('chessboard_w', 7)

        # Subscribe to the camera topic
        self.flag = False
        self.create_subscription(
            Image,
            '/sim_camera/image_raw',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.img = np.zeros((1, 1, 3))

        # Create a client for the gazebo set_entity_state service
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        else:
            self.get_logger().info('Service found')
        self.req = SetEntityState.Request()
        self.send_pose_change()

        # Create a timer for calibration steps
        self.timer = self.create_timer(0.5, self.send_pose_change)
        self.timer = self.create_timer(1, self.calibration_step)

        # Set calibration parameters
        self.cb = (
            self.get_parameter('chessboard_h').get_parameter_value().integer_value,
            self.get_parameter('chessboard_w').get_parameter_value().integer_value
        )
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.get_logger().info(f"Starting calibration for a %dx%d chessboard over %d images!" % (self.cb[0], self.cb[1], self.num_images))

        # Create CVBridge instance
        self.bridge = CvBridge()

        # termination self.criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.cb[0]*self.cb[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.cb[0],0:self.cb[1]].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
            
    def calibration_step(self):
        self.get_logger().info("Attempting...")
        if not self.flag or self.num_images <= 0: return

        gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        self.gray = gray

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (self.cb[0], self.cb[1]), None)

        # If found, add object points, image points (after refining them)
        if ret:
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners)
            self.num_images -= 1
            self.get_logger().info(f"%d images remaining" % self.num_images)

            # Draw and display the corners
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
            cv.drawChessboardCorners(self.img, (self.cb[0], self.cb[1]), corners2, ret)
            self.flag = False

        cv.imshow('img', self.img)
        cv.waitKey(1)

        if self.num_images <= 0: self.finalize()

    def send_pose_change(self):
        self.req.state = EntityState()

        self.req.state.name = 'chessboard'

        self.req.state.pose = Pose()
        self.req.state.pose.position.x =  1.5  + 3.5  * random()
        self.req.state.pose.position.y = -1.25 + 2.25 * random()
        self.req.state.pose.position.z =  0.5  + 1.0  * random()

        q = quaternion_from_euler(
            -pi/8 + pi/4* random() + 0.0, 
            -pi/8 + pi/4* random() + 0.0, 
            -pi/8 + pi/4* random() + pi/2
        )
        self.req.state.pose.orientation.x = q[0]
        self.req.state.pose.orientation.y = q[1]
        self.req.state.pose.orientation.z = q[2]
        self.req.state.pose.orientation.w = q[3]

        self.req.state.twist = Twist()

        self.req.state.reference_frame = 'world'

        self.resp = self.cli.call_async(self.req)

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.flag = True

    def finalize(self):
        path = os.path.join(get_package_share_directory('pose_estimation'), 'sim_calibration.json')
        cv.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)
        h, w = self.img.shape[:2]
        new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        with open(path, 'w') as f:
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

        self.get_logger().info(f"Finilizing... Saved to %s" % path)
        return mtx, dist, rvecs, tvecs, new_mtx, roi

def main(args=None):
    rclpy.init(args=args)
    viewer = Calibrator()

    while rclpy.ok() and viewer.num_images > 0:
        rclpy.spin_once(viewer)

    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
