from telnetlib import FORWARD_X
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.duration import Duration
from geometry_msgs.msg import Wrench
from rclpy.qos import QoSPresetProfiles

import numpy as np

# Flags
POS_X = 0b00000001
NEG_X = 0b00000010
POS_Y = 0b00000100
NEG_Y = 0b00001000
POS_Z = 0b00010000
NEG_Z = 0b00100000
POS_YAW = 0b01000000
NEG_YAW = 0b10000000


class ThrustController(Node):
    def __init__(self):
        super().__init__('thrust_controller')
        
        self.create_subscription(Int16, 'thrust_cmd', self.callback, QoSPresetProfiles.get_from_short_key('system_default'))
        
        self.pub = self.create_publisher(Wrench, 'chaser_0/gazebo_ros_force', QoSPresetProfiles.get_from_short_key('system_default'))
        
    def callback(self, flag):
        flag = flag.data
        
        req = Wrench()
        
        req.force.x = 2e0 * (1 if flag & POS_X else (-1 if flag & NEG_X else 0))
        req.force.y = 2e0 * (1 if flag & POS_Y else (-1 if flag & NEG_Y else 0))
        req.force.z = 2e0 * (1 if flag & POS_Z else (-1 if flag & NEG_Z else 0))
        req.torque.z = 5e-2 * (1 if flag & POS_YAW else (-1 if flag & NEG_YAW else 0))
        
        self.pub.publish(req)

    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()