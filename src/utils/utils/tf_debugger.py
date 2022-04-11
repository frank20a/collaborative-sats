import rclpy
from rclpy.time import Duration, Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSPresetProfiles

import numpy as np


class Test(Node):
    def __init__(self):
        super().__init__('tf_debugger')
        
        # create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
    
        # create a tf2 broadcaster
        self.pub_estim = self.create_publisher(TransformStamped, 'debug_estimation', QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.pub_filt = self.create_publisher(TransformStamped, 'debug_filtered', QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    
def main(args=None):
    rclpy.init(args=args)
    node = Test()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        # Estimated Pose
        try:
            t = node.buffer.lookup_transform('world', 'chaser_0/estimated_pose', Time(seconds=0))
            node.pub_estim.publish(t)
            # node.get_logger().info('{}:{}'.format((t.stamp.sec, t.stamp.nanosec)))
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        
        # Filtered Pose
        try:
            t = node.buffer.lookup_transform('world', 'chaser_0/filtered_estimation', Time(seconds=0))
            node.pub_filt.publish(t)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()