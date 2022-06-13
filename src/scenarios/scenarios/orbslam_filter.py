import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from rclpy.qos import QoSPresetProfiles
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.transform_broadcaster import TransformBroadcaster

import numpy as np

from .tf_utils import do_transform_pose

class ORBSLAMFilter(Node):
    def __init__(self):
        super().__init__('orbslam_filter')

        self.broadcaster = TransformBroadcaster(self)

        self.offset = [0.0, 0.0, 2.0]

        # Publisher(s)
        self.create_subscription(
            PoseStamped,
            'pose',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
    def callback(self, msg: PoseStamped):
        stamp = msg.header.stamp
        msg = msg.pose

        factor = 2.0

        req = TransformStamped()
        req.header.stamp = stamp
        req.header.frame_id = (self.get_namespace() + '/camera_optical').strip('/')
        req.child_frame_id = (self.get_namespace() + '/estimated_pose').strip('/')
        req.transform.translation.x = msg.position.x * factor + self.offset[0]
        req.transform.translation.y = msg.position.y * factor + self.offset[1]
        req.transform.translation.z = msg.position.z * factor + self.offset[2]
        req.transform.rotation.x = msg.orientation.x
        req.transform.rotation.y = msg.orientation.y
        req.transform.rotation.z = msg.orientation.z
        req.transform.rotation.w = msg.orientation.w

        # self.get_logger().info('Publishing transform: {}'.format(req))

        self.broadcaster.sendTransform(req)
        

def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAMFilter()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()