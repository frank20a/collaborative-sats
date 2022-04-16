from telnetlib import FORWARD_X
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.duration import Duration
from geometry_msgs.msg import Wrench
from rclpy.qos import QoSPresetProfiles

import numpy as np

from .parameters import force, torque
from .flags import *


class ThrustController(Node):
    def __init__(self):
        super().__init__('thrust_controller')
        
        self.declare_parameter('verbose', 0)
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        self.create_subscription(Int16, 'thrust_cmd', self.callback, QoSPresetProfiles.get_from_short_key('system_default'))
        
        self.pub = self.create_publisher(Wrench, 'gazebo_ros_force', QoSPresetProfiles.get_from_short_key('system_default'))
        
    def callback(self, flag):
        flag = flag.data
        if self.verbose > 0: 
            self.get_logger().info('{0:012b}'.format(flag))
        
        req = Wrench()
        
        req.force.x = force * (1 if flag & POS_X else (-1 if flag & NEG_X else 0))
        req.force.y = force * (1 if flag & POS_Y else (-1 if flag & NEG_Y else 0))
        req.force.z = force * (1 if flag & POS_Z else (-1 if flag & NEG_Z else 0))
        req.torque.x = torque * (1 if flag & POS_ROLL else (-1 if flag & NEG_ROLL else 0))
        req.torque.y = torque * (1 if flag & POS_PITCH else (-1 if flag & NEG_PITCH else 0))
        req.torque.z = torque * (1 if flag & POS_YAW else (-1 if flag & NEG_YAW else 0))
        
        self.pub.publish(req)

    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()