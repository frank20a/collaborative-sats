from cv2 import VideoCapture
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')


        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_raw', QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.cam = VideoCapture(0)
        self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        ret, frame = self.cam.read()
        self.pub.publish(self.bridge.cv2_to_imgmsg(np.asarray(frame), 'bgr8'))
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()