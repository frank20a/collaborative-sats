import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3, Pose, Wrench
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse
from .tf_utils import get_pose_diff, odometry2array, odometry2pose, vector_rotate_quaternion

import numpy as np
from simple_pid import PID

from .flags import *


def stepify(x, up_thresh = 0.25, down_thresh = None):
    if down_thresh is None:
        down_thresh = -up_thresh
    
    if x > up_thresh: return 1
    if x < down_thresh: return -1
    return 0


class PIDController(Node):
    def __init__(self):
        super().__init__('pid')
        self.declare_parameter('verbose', 0)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        self.setpoint = Pose()
        self.setpoint.position.x = -1.0
        self.setpoint.position.y = -1.0
        self.setpoint.position.z = 0.75
        q = quaternion_from_euler(0.0, 0.0, np.pi)
        self.setpoint.orientation.x = q[0]
        self.setpoint.orientation.y = q[1]
        self.setpoint.orientation.z = q[2]
        self.setpoint.orientation.w = q[3]

        self.create_subscription(
            Pose,
            'setpoint',
            self.set_setpoint,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        if self.thruster_type == 'onoff':
            self.publisher = self.create_publisher(Int16, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        elif self.thruster_type == 'pwm':
            self.publisher = self.create_publisher(Wrench, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
            
        
        if self.verbose > 1:
            self.debug_setpoint_xyz = self.create_publisher(Vector3, 'debug/setpoint_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_setpoint_rpy = self.create_publisher(Vector3, 'debug/setpoint_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))
            if self.thruster_type == 'onoff':
                self.debug_cmd_xyz = self.create_publisher(Vector3, 'debug/cmd_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
                self.debug_cmd_rpy = self.create_publisher(Vector3, 'debug/cmd_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))

    def callback(self, msg: Odometry):
        
        # Get difference to setpoint
        target = get_pose_diff(self.setpoint, odometry2pose(msg))
        
        # Transform to euler coordinates
        euler = euler_from_quaternion([target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        pose = np.array([target.position.x, target.position.y, target.position.z, euler[0], euler[1], euler[2]])
        
        
        # Publish debug messages
        if self.verbose > 1:
            
            tmp = Vector3()
            tmp.x = self.setpoint.position.x
            tmp.y = self.setpoint.position.y
            tmp.z = self.setpoint.position.z
            self.debug_setpoint_xyz.publish(tmp)
            
            tmp = Vector3()
            euler = euler_from_quaternion([self.setpoint.orientation.x, self.setpoint.orientation.y, self.setpoint.orientation.z, self.setpoint.orientation.w])
            tmp.x = euler[0] * 180 / np.pi
            tmp.y = euler[1] * 180 / np.pi
            tmp.z = euler[2] * 180 / np.pi
            self.debug_setpoint_rpy.publish(tmp)
            
            if self.thruster_type == 'onoff':
                cmd = '{0:012b}'.format(cmd.data)
                tmp = Vector3()
                tmp.x = 1.0 if cmd[0] == '1' else (-1.0 if cmd[1] == '1' else 0.0)
                tmp.y = 1.0 if cmd[2] == '1' else (-1.0 if cmd[3] == '1' else 0.0)
                tmp.z = 1.0 if cmd[4] == '1' else (-1.0 if cmd[5] == '1' else 0.0)
                self.debug_cmd_xyz.publish(tmp)
                
                tmp = Vector3()
                tmp.x = 1.0 if cmd[6] == '1' else (-1.0 if cmd[7] == '1' else 0.0)
                tmp.y = 1.0 if cmd[8] == '1' else (-1.0 if cmd[9] == '1' else 0.0)
                tmp.z = 1.0 if cmd[10] == '1' else (-1.0 if cmd[11] == '1' else 0.0)

    def set_setpoint(self, msg: Pose):
        self.setpoint = msg
        

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()