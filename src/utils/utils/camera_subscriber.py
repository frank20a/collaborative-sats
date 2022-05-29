import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.bridge = CvBridge()
        self.create_subscription(Image, 'camera/color/image_raw', self.callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    def callback(self, msg):
        self.get_logger().info("Test")
        cv.imshow('Camera Subscriber', self.bridge.imgmsg_to_cv2(msg))
        cv.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()