import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion

import numpy as np
from simple_pid import PID

# Flags
POS_X = 0b00000001
NEG_X = 0b00000010
POS_Y = 0b00000100
NEG_Y = 0b00001000
POS_Z = 0b00010000
NEG_Z = 0b00100000
POS_YAW = 0b01000000
NEG_YAW = 0b10000000
flags = [1, 2, 4, 8, 16, 32, 0, 0, 0, 0, 64, 128]

def stepify(x, up_thresh = 0.25, down_thresh = None):
    if down_thresh is None:
        down_thresh = -up_thresh
    
    if x > up_thresh: return 1
    if x < down_thresh: return -1
    return 0


class Odometry2TF(Node):
    def __init__(self):
        super().__init__('pid')
        self.declare_parameter('setpoint', '-1 0.5 0.75 0 0 0')
        self.declare_parameter('verbose', 2)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        # self.setpoint = [float(i) for i in self.get_parameter('setpoint').get_parameter_value().string_value.split()]
        self.setpoint = [1.0, 1.0, 0.75, 0.0, 0.0, np.pi]
        
        self.controller = [
            PID(80, 10, 200, output_limits=(-1, 1), sample_time=1/30.0),  # x
            PID(80, 10, 200, output_limits=(-1, 1), sample_time=1/30.0),  # y
            PID(80, 10, 200, output_limits=(-1, 1), sample_time=1/30.0),  # z
            PID(75, 15, 80, output_limits=(-1, 1), sample_time=1/30.0),  # roll
            PID(75, 15, 80, output_limits=(-1, 1), sample_time=1/30.0),  # pitch
            PID(75, 15, 80, output_limits=(-1, 1), sample_time=1/30.0),  # yaw
        ]

        self.create_subscription(
            Twist,
            'chaser_0/setpoint',
            self.set_setpoint,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.create_subscription(
            Odometry,
            'chaser_0/odom',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.publisher = self.create_publisher(Int16, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        
        if self.verbose > 1:
            self.debug_setpoint_xyz = self.create_publisher(Vector3, 'debug/setpoint_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_setpoint_rpy = self.create_publisher(Vector3, 'debug/setpoint_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_cmd_xyz = self.create_publisher(Vector3, 'debug/cmd_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_cmd_rpy = self.create_publisher(Vector3, 'debug/cmd_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))

    def callback(self, msg):
        cmd = Int16()
        cmd.data = 0
        
        euler = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        target = np.array([
            msg.pose.pose.position.x, 
            msg.pose.pose.position.y, 
            msg.pose.pose.position.z, 
            euler[0], 
            euler[1], 
            euler[2]
        ]) - np.array(self.setpoint)
        
        pose = np.array([
            target[0], 
            target[1], 
            target[2],
            target[3],
            target[4],
            target[5]
        ])
        
        for i in range(6):
            # Skip roll and pitch control
            if i == 3 or i == 4: continue
            
            tmp = stepify(self.controller[i](pose[i]))
            if tmp != 0:
                cmd.data |= flags[2 * i + (1 if tmp < 0 else 0)]
        
        if self.verbose > 0: self.get_logger().info('{0:08b}'.format(cmd.data))
        self.publisher.publish(cmd)
        
        
        if self.verbose > 1:
            cmd = '{0:08b}'.format(cmd.data)
            
            tmp = Vector3()
            tmp.x = self.setpoint[0]
            tmp.y = self.setpoint[1]
            tmp.z = self.setpoint[2]
            self.debug_setpoint_xyz.publish(tmp)
            
            tmp = Vector3()
            tmp.x = self.setpoint[3] * 180 / np.pi
            tmp.y = self.setpoint[4] * 180 / np.pi
            tmp.z = self.setpoint[5] * 180 / np.pi
            self.debug_setpoint_rpy.publish(tmp)
            
            tmp = Vector3()
            tmp.x = 1.0 if cmd[0] == '1' else (-1.0 if cmd[1] == '1' else 0.0)
            tmp.y = 1.0 if cmd[2] == '1' else (-1.0 if cmd[3] == '1' else 0.0)
            tmp.z = 1.0 if cmd[4] == '1' else (-1.0 if cmd[5] == '1' else 0.0)
            self.debug_cmd_xyz.publish(tmp)
            
            tmp = Vector3()
            tmp.x = 0.0
            tmp.y = 0.0
            tmp.z = 1.0 if cmd[6] == '1' else (-1.0 if cmd[7] == '1' else 0.0)
            self.debug_cmd_rpy.publish(tmp)           

    def set_setpoint(self, msg):
        self.setpoint = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
    

def main(args=None):
    rclpy.init(args=args)
    node = Odometry2TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()