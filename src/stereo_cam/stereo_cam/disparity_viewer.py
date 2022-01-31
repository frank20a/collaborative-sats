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

def nothing(args):pass

# Setting the updated parameters before computing disparity map
cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)

cv2.createTrackbar('numDisparities','disp',1,17,nothing)
cv2.createTrackbar('blockSize','disp',5,50,nothing)
cv2.createTrackbar('preFilterType','disp',1,1,nothing)
cv2.createTrackbar('preFilterSize','disp',2,25,nothing)
cv2.createTrackbar('preFilterCap','disp',5,62,nothing)
cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
cv2.createTrackbar('speckleRange','disp',0,100,nothing)
cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
cv2.createTrackbar('disp12MaxDiff','disp',5,25,nothing)
cv2.createTrackbar('minDisparity','disp',5,25,nothing)


class ImageViewer(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")
        
        self.prev_t = time()

        self.img_r = None
        self.img_l = None
        
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

        self.create_timer(1/30, self.disparity)

    def l_callback(self, msg):
        self.img_l = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)

    def r_callback(self, msg):
        self.img_r = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2GRAY)
    
    def disparity(self):
        numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
        blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
        preFilterType = cv2.getTrackbarPos('preFilterType','disp')
        preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')*2 + 5
        preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
        textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
        uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
        speckleRange = cv2.getTrackbarPos('speckleRange','disp')
        speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
        disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
        minDisparity = cv2.getTrackbarPos('minDisparity','disp')

        stereo.setNumDisparities(numDisparities)
        stereo.setBlockSize(blockSize)
        stereo.setPreFilterType(preFilterType)
        stereo.setPreFilterSize(preFilterSize)
        stereo.setPreFilterCap(preFilterCap)
        stereo.setTextureThreshold(textureThreshold)
        stereo.setUniquenessRatio(uniquenessRatio)
        stereo.setSpeckleRange(speckleRange)
        stereo.setSpeckleWindowSize(speckleWindowSize)
        stereo.setDisp12MaxDiff(disp12MaxDiff)
        stereo.setMinDisparity(minDisparity)

        if self.img_l is not None and self.img_r is not None:
            # CAMERAS ARE OPPOSITE GAMW TO SYMPAN OLAKERO
            disparity = stereo.compute(self.img_r, self.img_l).astype(np.float32)
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
