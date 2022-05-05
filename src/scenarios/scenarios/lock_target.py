import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose
from rclpy.qos import QoSPresetProfiles
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from tf_transformations import quaternion_from_euler

from .utils import tf2pose

class TargetLock(Node):
    def __init__(self):
        super().__init__('target_lock')

        # Declare parameters
        self.declare_parameter('frequency', 5.0)

        self.freq = self.get_parameter('frequency').get_parameter_value().double_value
        
        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        # Positions
        self.target_pose = None
        self.chaser_pose = None

        # Timer for the reference pose calculator
        self.create_timer(1 / self.freq, self.callback)

        # Publisher(s)
        self.setpoint_pub = self.create_publisher(
            Pose,
            'setpoint',
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
    def callback(self):
        if self.target_pose is None or self.chaser_pose is None:
            return

        req = Pose()
        req.position.x = self.chaser_pose.position.x
        req.position.y = self.chaser_pose.position.y
        req.position.z = self.chaser_pose.position.z
        # self.get_logger().info('req: {}'.format(self.target_pose))

        eul = [
            0.0,
            np.arctan2(- self.target_pose.position.z + self.chaser_pose.position.z, np.sqrt((self.target_pose.position.x - self.chaser_pose.position.x)**2 + (self.target_pose.position.y - self.chaser_pose.position.y)**2)),
            np.arctan2(self.target_pose.position.y - self.chaser_pose.position.y, self.target_pose.position.x - self.chaser_pose.position.x),
        ]
        q = quaternion_from_euler(eul[0], eul[1], eul[2])
        req.orientation.x = q[0]
        req.orientation.y = q[1]
        req.orientation.z = q[2]
        req.orientation.w = q[3]

        self.target_pose = None
        self.chaser_pose = None

        self.setpoint_pub.publish(req)
        

def main(args=None):
    rclpy.init(args=args)
    node = TargetLock()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.target_pose is None:
            try:
                node.target_pose = tf2pose(node.buffer.lookup_transform('world', (node.get_namespace() + '/estimated_pose').lstrip('/'), Time(seconds=0)))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        
        if node.chaser_pose is None:
            try:
                node.chaser_pose = tf2pose(node.buffer.lookup_transform('world', (node.get_namespace() + '/body').lstrip('/'), Time(seconds=0)))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()