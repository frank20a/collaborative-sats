import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_matrix

import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time

from .undistort import get_camera_calibration
from .objPoints import models
from .rigid_body_filters import filters

class ArucoBoardPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_board_estimator')
        self.bridge = CvBridge()
        self.pose_br = TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('verbose', 1)
        self.declare_parameter('sim', False)
        self.declare_parameter('model', 'marker_cube_1')
        self.declare_parameter('duration', False)
        self.declare_parameter('filter', '')
        self.declare_parameter('ra_len', 3)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.fps_flag = self.get_parameter('duration').get_parameter_value().bool_value
        self.filter = self.get_parameter('filter').get_parameter_value().string_value
        if self.filter == '': 
            self.filter = False
        else:
            self.filter = filters[self.filter](1/60)
        self.avg = np.zeros((self.get_parameter('ra_len').get_parameter_value().integer_value, 2, 3))
            

        # Setup ArUco recognition
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_50)
        self.params = cv.aruco.DetectorParameters_create()
        self.board = cv.aruco.Board_create(
            ids = np.array([[i] for i in range(6)], dtype=np.int32), 
            dictionary = self.dictionary, 
            objPoints = models[self.get_parameter('model').get_parameter_value().string_value]
        )

        # Get calibration parameters
        if self.get_parameter('sim').get_parameter_value().bool_value:
            self.mtx, self.dist, _, _, _, _ = get_camera_calibration('sim_calibration.json')
        else:
            self.mtx, self.dist, _, _, _, _ = get_camera_calibration()

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
        
        self.to_filter = self.create_publisher(
            TransformStamped,
            'estimated_pose',
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.debug_pub = self.create_publisher(
            Vector3,
            'raw_estimation',
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
                       
    def callback(self, msg: Image):
        tt = time()
        
        # Read image and undistort
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        t = self.get_estimation(img)
        if t is None: return
        t.header.stamp = msg.header.stamp
        
        # Send the transform
        self.pose_br.sendTransform(t)
        self.to_filter.publish(t)
        
        if self.fps_flag:
            self.get_logger().info('ArUco duration: %.3f ms' % ((time() - tt) * 1e3))

    def get_estimation(self, img) -> TransformStamped:
        # ======================== Detect Markers ========================
        corners, ids, _ = cv.aruco.detectMarkers(img, self.dictionary, parameters=self.params)
        if len(corners) <= 0: return None
        
        ids = ids.flatten()

        ret, rvec, tvec = cv.aruco.estimatePoseBoard(
            corners, 
            ids, 
            self.board, 
            self.mtx, 
            self.dist,
            np.zeros((1, 3)),
            np.zeros((1, 3)),
        )
        if not ret: return None
            
        rvec = np.reshape(rvec, (1,3))
        tvec = np.reshape(tvec, (1,3))
        
        # ======================== Rolling Average =======================
        self.avg = np.roll(self.avg, -1, axis=0)
        self.avg[-1, :, :] = np.array([rvec, tvec]).reshape((2, 3))
        rvec = np.average(self.avg[:, 0, :], axis=0)
        tvec = np.average(self.avg[:, 1, :], axis=0)

        # ========================= Make Message =========================
        t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = (self.get_namespace() + '/camera_optical').lstrip('/')
        t.child_frame_id = (self.get_namespace() + '/estimated_pose').lstrip('/')

        # Store the translation (i.e. position) information
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]

        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3], _ = cv.Rodrigues(rvec)
        quat = quaternion_from_matrix(rotation_matrix)
        
        
        # Quaternion format     
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # =========================== Debug ===========================
        tmp = Vector3()
        tmp.x = rvec[0]
        tmp.y = rvec[1]
        tmp.z = rvec[2]
        self.debug_pub.publish(tmp)
        
        # ========================== Verbose ==========================
        if self.verbose > 0:
            cv.aruco.drawAxis(img, self.mtx, self.dist, rvec, tvec, 0.1)
            cv.aruco.drawDetectedMarkers(img, corners, ids)
        
            img_msg = self.bridge.cv2_to_imgmsg(img)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = (self.get_namespace() + '/camera_optical').lstrip('/')
            self.publisher_disp.publish(img_msg)
            
        if self.verbose > 1:
            cv.imshow('Aruco Pose Estimation', img)
            cv.waitKey(1)
        
        if self.verbose > 2:
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

        return t   
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ArucoBoardPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()