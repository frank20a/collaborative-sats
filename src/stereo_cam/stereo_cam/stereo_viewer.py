import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

import cv2
from time import time
import numpy as np

bridge = CvBridge()

class ImageViewer(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")
        
        self.fps_l = 0
        self.fps_r = 0
        self.frame_count_l = 0
        self.frame_count_r = 0
        self.prev_t_l = time()
        self.prev_t_r = time()

        self.img_l = np.zeros((480, 720))
        self.img_r = np.zeros((480, 720))
        
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

        self.create_timer(1/15, self.disp_images)

    def l_callback(self, msg):
        self.img_l = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)


        self.frame_count_l += 1
        if self.frame_count_l % 30 == 0:
            self.fps_l = 30 / (time() - self.prev_t_l)
            self.prev_t_l = time()

    def r_callback(self, msg):
        self.img_r = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)

        self.frame_count_r += 1
        if self.frame_count_r % 30 == 0:
            self.fps_r = 30 / (time() - self.prev_t_r)
            self.prev_t_r = time()
    
    def disp_images(self):
        cv2.putText(self.img_l, "LEFT FPS: {0:d}".format(int(np.mean(self.fps_l))), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 3)
        cv2.putText(self.img_r, "RIGHT FPS: {0:d}".format(int(np.mean(self.fps_r))), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 3)

        img = np.concatenate((self.img_l, self.img_r), axis=1)

        cv2.imshow("Stereo camera", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
