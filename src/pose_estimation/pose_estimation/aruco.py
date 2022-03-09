import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2 as cv
from .undistort import get_camera_calibration
import numpy as np
from scipy.spatial.transform import Rotation as R


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')
        self.bridge = CvBridge()
        self.pose_br = TransformBroadcaster(self)

        # Setup ArUco recognition
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_50)
        self.params = cv.aruco.DetectorParameters_create()
        self.real_dim = 0.04

        # Get calibration parameters
        self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration()

        self.create_subscription(
            Image,
            '/undistorted',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        

    def callback(self, msg):
        # Read image and undistort
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Detect markers
        corners, ids, _ = cv.aruco.detectMarkers(img, self.dictionary, parameters=self.params)

        if len(corners) > 0:
            ids = ids.flatten()

            cv.aruco.drawDetectedMarkers(img, corners, ids)

            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, self.real_dim, self.mtx, self.dist)
            for rvec, tvec, id in zip(rvecs, tvecs, ids):
                # Source: https://automaticaddison.com/how-to-publish-tf-between-an-aruco-marker-and-a-camera/

                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera'
                t.child_frame_id = 'marker_' + str(id)
            
                # Store the translation (i.e. position) information
                t.transform.translation.x = tvec[0][0]
                t.transform.translation.y = tvec[0][1]
                t.transform.translation.z = tvec[0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv.Rodrigues(np.array(rvec[0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                t.transform.rotation.x = quat[0] 
                t.transform.rotation.y = quat[1] 
                t.transform.rotation.z = quat[2] 
                t.transform.rotation.w = quat[3] 
        
                # Send the transform
                self.pose_br.sendTransform(t)

                cv.aruco.drawAxis(img, self.mtx, self.dist, rvec, tvec, 0.05)


        cv.imshow('Aruco Pose Estimation', img)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    viewer = ArucoPoseEstimator()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()