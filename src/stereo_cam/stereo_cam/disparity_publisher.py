from sympy import diag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from stereo_msgs.msg import DisparityImage

import cv2
from time import time
import numpy as np

bridge = CvBridge()
stereo = cv2.StereoBM_create()

# Stereo parameters
numDisparities = 16 * 4
minDisparity = -10

stereo.setNumDisparities(numDisparities)
stereo.setBlockSize(25)
stereo.setPreFilterSize(7)
stereo.setPreFilterCap(20)
stereo.setUniquenessRatio(15)
stereo.setSpeckleRange(3)
stereo.setMinDisparity(minDisparity)

class DisparityViewer(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")

        self.img_r = None
        self.updated_r = False
        self.img_l = None
        self.updated_l = False

        max_fps = 60
        
        self.create_subscription(
            Image,
            '/stereo/left/image_raw',
            self.l_callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.create_subscription(
            Image,
            '/stereo/right/image_raw',
            self.r_callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.publisher_disp = self.create_publisher(
            DisparityImage, 
            'stereo/disparity', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        
        self.create_timer(1/max_fps, self.disparity)

    def l_callback(self, msg):
        self.img_l = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)
        self.updated_l = True

    def r_callback(self, msg):
        self.img_r = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)
        self.updated_r = True

    def disparity(self):
        # ----> Check if both images are fresh
        if not (self.updated_r and self.updated_l): return
        self.updated_l = False
        self.updated_r = False


        disparity = stereo.compute(self.img_l, self.img_r).astype(np.float32)
        disparity = (disparity/16.0 - (minDisparity-1))/numDisparities

        # ----> Send disparsity image message
        disp_msg = DisparityImage()
        disp_msg.max_disparity = 1.0
        disp_msg.min_disparity = 0.0
        disp_msg.delta_d = 1.0 / numDisparities
        disp_msg.image = bridge.cv2_to_imgmsg(disparity)
        disp_msg.t = 0.065
        disp_msg.f = (720 / 2) / np.tan(1.04699999 / 2)
        self.publisher_disp.publish(disp_msg)


def main(args=None):
    rclpy.init(args=args)
    viewer = DisparityViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
