from telnetlib import FORWARD_X
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.duration import Duration
from gazebo_msgs.srv import ApplyLinkWrench
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
        
        self.cli = self.create_client(ApplyLinkWrench, 'apply_link_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ApplyLinkWrench.Request()
        
        self.create_subscription(Int16, 'thrust', self.callback, QoSPresetProfiles.get_from_short_key('system_default'))
        
    def callback(self, flag):
        flag = flag.data
        
        self.req.link_name = 'chaser_0::body'
        # self.req.reference_frame = 'chaser_0::body'
        
        self.req.reference_point.x = 0.0
        self.req.reference_point.y = 0.0
        self.req.reference_point.z = 0.0
        
        self.req.wrench.force.x = 5e-2 * (1 if flag & POS_X else (-1 if flag & NEG_X else 0))
        self.req.wrench.force.y = 5e-2 * (1 if flag & POS_Y else (-1 if flag & NEG_Y else 0))
        self.req.wrench.force.z = 5e-2 * (1 if flag & POS_Z else (-1 if flag & NEG_Z else 0))
        self.req.wrench.torque.z = 2e-3 * (1 if flag & POS_YAW else (-1 if flag & NEG_YAW else 0))
        
        self.req.duration = Duration(seconds = -1).to_msg()
        
        return self.cli.call_async(self.req)

    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)
    # future = node.send_request()

    # while rclpy.ok():
    #     rclpy.spin_once(node)
    #     if future.done():
    #         try:
    #             response = future.result()
    #         except Exception as e:
    #             node.get_logger().info('Service call failed %r' % (e,))
    #         else:
    #             node.get_logger().info('Call was made. Successful: %r' % (response.success,))
    #         break 
    # node.get_logger().info(str(response.status_message))
        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()