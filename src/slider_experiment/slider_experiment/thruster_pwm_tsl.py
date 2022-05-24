import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from rclpy.qos import QoSPresetProfiles
from ament_index_python import get_package_share_directory

import numpy as np
import sys, os

from .parameters import force
from .flags import flags


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

        sys.path.insert(1, os.path.join(get_package_share_directory('slider_experiment'), 'python_build/tsl_optimizer'))
        import tsl_optimizer as optimizer
        self.solver = optimizer.solver()
        
        self.signals = [create_pwm(0, self.resolution) for i in range(8)]
        self.i = 0
        
        self.create_subscription(Vector3, 'thrust_cmd', self.callback, QoSPresetProfiles.get_from_short_key('system_default'))
        self.pub = self.create_publisher(Int16, 'thruster_flags', QoSPresetProfiles.get_from_short_key('sensor_data'))

        self.create_timer(1/(self.frequency * self.resolution), self.send_signals)
        
    def callback(self, msg: Vector3):
        
        T = self.solver.run(p = [msg.x, msg.y, msg.z]).solution

        if self.verbose > 0: 
            self.get_logger().info(f'\n Fx = {msg.x: 2.2f}\n Fy = {msg.y: 2.2f}\ntau = {msg.z: 2.2f}')
            self.get_logger().info(f'cmd: {T}')

        self.signals = [create_pwm(T[i] / force, self.resolution) for i in range(8)]
        
    def send_signals(self):
        req = Int16()
        
        tmp = 0
        for i in range(8):
            if self.signals[i][self.i] == 1:
                tmp ^= flags[i]
        try:
            req.data = tmp
        except AssertionError:
            print(tmp)

        
        self.i += 1
        self.i %= self.resolution
        
        self.pub.publish(req)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()