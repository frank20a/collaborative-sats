import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3, Pose
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .tf_utils import get_pose_diff, odometry2array, odometry2tfstamped, odometry2pose, vector_rotate_quaternion
from .tf2_geometry_msgs import do_transform_point

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


class PIDController(Node):
    def __init__(self):
        super().__init__('pid')
        self.declare_parameter('verbose', 2)
        
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
        # self.set_setpoint(self.setpoint)
        
        self.controller = []
        self.controller = self.controller + [PID(80, 10, 200, output_limits=(-1, 1), sample_time=1/30.0) for i in range(3)]     # x, y, z
        self.controller = self.controller + [PID(75, 15, 80 , output_limits=(-1, 1), sample_time=1/30.0) for i in range(3)]     # roll, pitch, yaw

        self.create_subscription(
            Pose,
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

    def callback(self, msg: Odometry):
        cmd = Int16()
        cmd.data = 0
        
        # Get difference to setpoint
        target = get_pose_diff(self.setpoint, odometry2pose(msg))
        
        # Transform to euler coordinates
        euler = euler_from_quaternion([target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        pose = np.array([target.position.x, target.position.y, target.position.z, euler[0], euler[1], euler[2]])
        
        # Update PID controllers
        control = np.zeros(6)
        for i in range(6):            
            control[i] = self.controller[i](pose[i])
        u = np.array(control)
        u[0:3] = vector_rotate_quaternion(u[0:3], odometry2array(msg)[3:])
        
        for i, f in enumerate(u):
            # Skip roll and pitch control
            if i == 3 or i == 4: continue
            
            tmp = stepify(f)
            if tmp != 0:
                cmd.data |= flags[2 * i + (1 if tmp < 0 else 0)]
        
        
        # Send thruster command
        if self.verbose > 0: self.get_logger().info('{0:08b}'.format(cmd.data))
        self.publisher.publish(cmd)
        
        # Publish debug messages
        if self.verbose > 1:
            cmd = '{0:08b}'.format(cmd.data)
            
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

    def set_setpoint(self, msg: Pose):
        # self.setpoint = [msg.position.x, msg.position.y, msg.position.z] + euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        # for i, pid in zip(self.setpoint, self.controller):
        #     pid.setpoint = i
        
        self.setpoint = msg
        

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()