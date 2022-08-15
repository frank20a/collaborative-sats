import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix

import cv2 as cv
import numpy as np

def create_pwm(value, resolution):
    if value < 0.0: 
        value = -value
    if value > 1.0:
        value = 1.0
        
    return np.concatenate((np.ones(np.floor(resolution * value).astype(np.int32)), np.zeros(np.ceil(resolution * (1 - value)).astype(np.int32))))


class PoseRaw(Node):
    def __init__(self):
        super().__init__('pose_raw_converter')

        self.pose_br = TransformBroadcaster(self)

        self.create_subscription(Twist, 'pose_raw', self.callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    def callback(self, msg: Twist):
        # self.get_logger().info("Test")

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = (self.get_namespace() + '/camera_optical').lstrip('/')
        t.child_frame_id = (self.get_namespace() + '/estimated_pose').lstrip('/')

        # Store the translation (i.e. position) information
        t.transform.translation.x = msg.linear.x
        t.transform.translation.y = msg.linear.y
        t.transform.translation.z = msg.linear.z

        # Store the rotation information
        rvec = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3], _ = cv.Rodrigues(rvec)
        quat = quaternion_from_matrix(rotation_matrix)
        
        
        # Quaternion format     
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.pose_br.sendTransform(t)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = PoseRaw()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()