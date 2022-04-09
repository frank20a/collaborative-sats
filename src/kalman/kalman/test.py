import rclpy
from rclpy.time import Duration, Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np


class Test(Node):
    def __init__(self):
        super().__init__('tester_node')
        
        # create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = Test()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        try:
            t = node.buffer.lookup_transform('world', 'chaser_0/estimated_pose', Time(nanoseconds=0))
            node.get_logger().info('{}:{}'.format((t.stamp.sec, t.stamp.nanosec)))
        except LookupException:
            pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()