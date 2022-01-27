import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2 as cv
from time import time
import numpy as np

bridge = CvBridge()

class ImageViewer(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")
        
        self.fps_l = list(np.zeros(3))
        self.fps_r = list(np.zeros(3))
        self.prev_t_l = time()
        self.prev_t_r = time()
        
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

    def r_callback(self, msg):
        self.fps_r = [1 / (time() - self.prev_t_r)] + self.fps_l[0:2]
        self.prev_t_r = time()
        # print("RIGHT FPS: {0:d}".format(int(np.mean(self.fps_r))))
        cv.imshow("Right camera", bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        cv.waitKey(3)

    def l_callback(self, msg):
        self.fps_l = [1 / (time() - self.prev_t_l)] + self.fps_l[0:2]
        self.prev_t_l = time()
        # print("LEFT FPS: {0:d}".format(int(np.mean(self.fps_l))))
        cv.imshow("Left camera", bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        cv.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
