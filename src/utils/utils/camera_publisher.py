import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.declare_parameter('verbose', 0)
        self.declare_parameter('camera_type', 'builtin')
        self.declare_parameter('camera_ip', '83.233.159.27:8080')
        self.declare_parameter('fps', 10)
        self.declare_parameter('camera_id', 0)

        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        self.camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_raw', QoSPresetProfiles.get_from_short_key('sensor_data'))

        if self.camera_type == 'builtin':
            self.cam = cv.VideoCapture(self.camera_id)
        elif self.camera_type == 'ip':
            self.cam = cv.VideoCapture('rtsp://' + self.camera_ip + '/h264_pcm.sdp')
            self.get_logger().info("Connected to camera at %s" % self.camera_ip)
            self.cam.set(cv.CAP_PROP_FPS, self.fps)
            self.cam.set(cv.CAP_PROP_BUFFERSIZE, 2)

        self.create_timer(1 / self.fps, self.timer_callback)
        
    def timer_callback(self):
        ret, frame = self.cam.read()

        if not ret:
            return

        if self.verbose:
            cv.imshow('frame', frame)
            cv.waitKey(1)

        self.pub.publish(self.bridge.cv2_to_imgmsg(np.asarray(frame), 'bgr8'))
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()