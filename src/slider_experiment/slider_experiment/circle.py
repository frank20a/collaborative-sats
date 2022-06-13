import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSPresetProfiles
from tf_transformations import quaternion_from_euler

import numpy as np


class ThrustController(Node):
    def __init__(self):
        super().__init__('circler')
        
        self.declare_parameter('verbose', 0)
        self.declare_parameter('period', 1.0)

        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.period = self.get_parameter('period').get_parameter_value().double_value

        self.r = 0.3
        self.theta = 0.0

        self.create_timer(self.period, self.callback)
        self.pub = self.create_publisher(PoseStamped, '/slider_0/relative_setpoint', QoSPresetProfiles.get_from_short_key('system_default'))
        
    def callback(self):
        
        self.theta += 2*np.pi/60
        self.theta %= 2*np.pi

        res = Pose()
        res.position.x = self.r * np.cos(self.theta)
        res.position.y = self.r * np.sin(self.theta)
        res.position.z = 0.0
        tmp = quaternion_from_euler(0.0, 0.0, 0.0)
        res.orientation.x = tmp[0]
        res.orientation.y = tmp[1]
        res.orientation.z = tmp[2]
        res.orientation.w = tmp[3]

        if self.verbose > 0:
            self.get_logger().info(f'\n x = {res.position.x: 2.2f}\n y = {res.position.y: 2.2f}')

        msg = PoseStamped()
        msg.pose = res
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)       
    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()