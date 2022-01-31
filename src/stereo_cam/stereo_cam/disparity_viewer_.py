import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2
from time import time
import numpy as np

bridge = CvBridge()
stereo = cv2.StereoBM_create()

# Setting the updated parameters before computing disparity map
minDisparity = 0
numDisparities = 16 * 4
stereo.setMinDisparity(minDisparity)
stereo.setNumDisparities(numDisparities)
stereo.setBlockSize(9)
stereo.setUniquenessRatio(5)
stereo.setSpeckleRange(6)
stereo.setSpeckleWindowSize(15)
stereo.setDisp12MaxDiff(10)

class ImageViewer(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")
        
        self.prev_t = time()

        self.img_r = None
        self.img_l = None
        
        self.create_subscription(
            Image,
            '/camera_l/image_raw',
            self.l_callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.create_subscription(
            Image,
            '/camera_r/image_raw',
            self.r_callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        self.create_timer(1/30, self.disparity)

    def l_callback(self, msg):
        self.img_l = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)

    def r_callback(self, msg):
        self.img_r = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)
    
    def disparity(self):
        if self.img_l is not None and self.img_r is not None:
            disparity = stereo.compute(self.img_l, self.img_r).astype(np.float32)
            disparity = (disparity/16.0 - minDisparity)/numDisparities

            cv2.imshow("disp",disparity)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
