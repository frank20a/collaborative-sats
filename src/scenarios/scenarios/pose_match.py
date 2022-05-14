import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose
from rclpy.qos import QoSPresetProfiles
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

from .tf_utils import do_transform_pose

class PoseMatch(Node):
    def __init__(self):
        super().__init__('pose_match')

        # Declare parameters
        self.declare_parameter('frequency', 5.0)

        self.freq = self.get_parameter('frequency').get_parameter_value().double_value
        
        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        # Positions
        self.target_tf = None

        # Timer for the reference pose calculator
        self.create_timer(1 / self.freq, self.callback)

        # Publisher(s)
        self.setpoint_pub = self.create_publisher(
            Pose,
            'setpoint',
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
    def callback(self):
        if self.target_tf is None: return
        
        ref = Pose()
        ref.position.x = -1.0

        ref = do_transform_pose(ref, self.target_tf)
        # self.get_logger().info('ref: {}'.format(self.target_tf))

        self.setpoint_pub.publish(ref)

        self.target_tf = None
        

def main(args=None):
    rclpy.init(args=args)
    node = PoseMatch()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.target_tf is None:
            try:
                node.target_tf = node.buffer.lookup_transform('world', ('/estimated_pose').lstrip('/'), Time(seconds=0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()