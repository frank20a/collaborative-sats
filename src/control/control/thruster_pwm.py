import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rclpy.qos import QoSPresetProfiles

import numpy as np

from .parameters import force, torque
from .flags import *


def create_pwm(value, resolution):
    if value < 0.0: 
        value = -value
    if value > 1.0:
        value = 1.0
        
    return np.concatenate((np.ones(np.floor(resolution * value).astype(np.int32)), np.zeros(np.ceil(resolution * (1 - value)).astype(np.int32))))


class ThrustController(Node):
    def __init__(self):
        super().__init__('thrust_controller')
        
        self.declare_parameter('verbose', 0)
        self.declare_parameter('frequency', 10)
        self.declare_parameter('resolution', 100)
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().integer_value
        
        self.signals = [create_pwm(0, self.resolution) for i in range(6)]
        self.i = 0
        
        self.create_subscription(Wrench, 'thrust_cmd', self.callback, QoSPresetProfiles.get_from_short_key('system_default'))
        self.pub = self.create_publisher(Wrench, 'chaser_0/gazebo_ros_force', QoSPresetProfiles.get_from_short_key('system_default'))
        self.create_timer(1/(self.frequency * self.resolution), self.send_signals)
        if self.verbose > 1: 
            self.debug_cmd_xyz = self.create_publisher(Vector3, 'debug/cmd_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_cmd_rpy = self.create_publisher(Vector3, 'debug/cmd_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    def callback(self, msg):
        if self.verbose > 0: 
            self.get_logger().info(f'{msg.force.x: 2.2f}, {msg.force.y: 2.2f}, {msg.force.z: 2.2f}, {msg.torque.x: 2.2f}, {msg.torque.y: 2.2f}, {msg.torque.z: 2.2f}')
        
        self.signals = [
            create_pwm(msg.force.x, self.resolution) * np.sign(msg.force.x),
            create_pwm(msg.force.y, self.resolution) * np.sign(msg.force.y),
            create_pwm(msg.force.z, self.resolution) * np.sign(msg.force.z),
            create_pwm(msg.torque.x, self.resolution) * np.sign(msg.torque.x),
            create_pwm(msg.torque.y, self.resolution) * np.sign(msg.torque.y),
            create_pwm(msg.torque.z, self.resolution) * np.sign(msg.torque.z)
        ]
        
    def send_signals(self):
        req = Wrench()
        
        req.force.x = force * self.signals[0][self.i]
        req.force.y = force * self.signals[1][self.i]
        req.force.z = force * self.signals[2][self.i]
        req.torque.x = torque * self.signals[3][self.i]
        req.torque.y = torque * self.signals[4][self.i]
        req.torque.z = torque * self.signals[5][self.i]
        
        self.i += 1
        self.i %= self.resolution
        
        self.pub.publish(req)
        
        if self.verbose > 1:
            tmp = Vector3()
            tmp.x = self.signals[0][self.i]
            tmp.y = self.signals[1][self.i]
            tmp.z = self.signals[2][self.i]
            self.debug_cmd_xyz.publish(tmp)
            
            tmp = Vector3()
            tmp.x = self.signals[3][self.i]
            tmp.y = self.signals[4][self.i]
            tmp.z = self.signals[5][self.i]
            self.debug_cmd_rpy.publish(tmp)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()