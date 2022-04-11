import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_matrix

import cv2 as cv
from .undistort import get_camera_calibration
import numpy as np
from scipy.spatial.transform import Rotation as R


def tf_msg_from_vecs(rvec, tvec, child_frame, parent_frame = 'camera_optical'):
    # Source: https://automaticaddison.com/how-to-publish-tf-between-an-aruco-marker-and-a-camera/

    # Create the coordinate transform
    t = TransformStamped()
    # t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    # Store the translation (i.e. position) information
    t.transform.translation.x = tvec[0][0]
    t.transform.translation.y = tvec[0][1]
    t.transform.translation.z = tvec[0][2]

    # Store the rotation information
    rotation_matrix = np.eye(4)
    rotation_matrix[0:3, 0:3], _ = cv.Rodrigues(np.array(rvec[0]))
    quat = quaternion_from_matrix(rotation_matrix)
    
    
    # Quaternion format     
    t.transform.rotation.x = quat[0] 
    t.transform.rotation.y = quat[1] 
    t.transform.rotation.z = quat[2] 
    t.transform.rotation.w = quat[3] 

    return t

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_estimator')
        self.bridge = CvBridge()
        self.pose_br = TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('verbose', 1)
        self.declare_parameter('marker_size', 0.12)
        self.declare_parameter('sim', False)

        # Setup ArUco recognition
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_50)
        self.params = cv.aruco.DetectorParameters_create()

        # Get calibration parameters
        if self.get_parameter('sim').get_parameter_value().bool_value:
            self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration('sim_calibration.json')
        else:
            self.mtx, self.dist, self.new_mtx, self.roi = get_camera_calibration()

        self.create_subscription(
            Image,
            'undistorted',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        self.publisher_disp = self.create_publisher(
            Image, 
            'aruco_verbose', 
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

            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 
                self.get_parameter('marker_size').get_parameter_value().double_value, 
                self.mtx, 
                self.dist
            )

            for rvec, tvec, id in zip(rvecs, tvecs, ids):
                t = tf_msg_from_vecs(
                    rvec, 
                    tvec, 
                    (self.get_namespace() + '/marker_' + str(id)).lstrip('/'), 
                    (self.get_namespace() + '/camera_optical').lstrip('/')
                )
                t.header.stamp = self.get_clock().now().to_msg()
        
                # Send the transform
                self.pose_br.sendTransform(t)
                if self.get_parameter('verbose').get_parameter_value().integer_value > 2:
                    # self.get_logger().info(f'\n%s\n%s' % (str(tvec), str(rvec)))
                    self.get_logger().info('\nTranslation: x={: >6.3f} y={: >6.3f} z={: >6.3f}\n   Rotation: x={: >6.3f} y={: >6.3f} z={: >6.3f} w={: >6.3f}'.format(
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ))

                cv.aruco.drawAxis(img, self.mtx, self.dist, rvec, tvec, 0.075)

        if self.get_parameter('verbose').get_parameter_value().integer_value > 0:
            img_msg = self.bridge.cv2_to_imgmsg(img)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = (self.get_namespace() + '/camera_optical').lstrip('/')

            self.publisher_disp.publish(img_msg)

        if self.get_parameter('verbose').get_parameter_value().integer_value > 1:
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