from pandas import Int16Dtype
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Vector3, Pose, Wrench, Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .tf_utils import odometry2pose, get_pose_diff, vector_rotate_quaternion, quaternion_inverse, odometry2array, get_state

import numpy as np
from numpy import pi
import os, sys

from .flags import *
from .parameters import force, torque, mpc_final_weights, mpc_input_weights, mpc_state_weights


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc')
        self.declare_parameter('verbose', 0)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        # self.declare_parameter('init_setpoint', '-1.0 0.5 0.75 0.0 0.0 -0.258819 0.9659258')
        self.declare_parameter('init_setpoint', '')
        init_setpoint = self.get_parameter('init_setpoint').get_parameter_value().string_value     
        
        if init_setpoint != '':
            tmp = [float(i) for i in init_setpoint.split(' ')]
            tmp = tmp[:3] + [0, 0, 0] + tmp[3:] + [0, 0, 0]
            self.setpoint = np.array(tmp, dtype=np.float64)
        else:
            q = quaternion_from_euler(0.0, 0.0, pi)
            self.setpoint = np.array([
                -1,         # x
                -1,         # y
                1,          # z
                0,          # x_dot
                0,          # y_dot
                0,          # z_dot 
                q[0],       # qx
                q[1],       # qy
                q[2],       # qz
                q[3],       # qw
                0,          # wx
                0,          # wy
                0           # wz
            ], dtype=np.float64)
        
        # Solver
        sys.path.insert(1, os.path.join(get_package_share_directory('control'), 'python_build/chaser_mpc'))
        import chaser_mpc
        self.solver = chaser_mpc.solver()       

        # Subscriptions
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
        
        # Publishers
        self.publisher = self.create_publisher(Wrench, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        
        if self.verbose > 1:
            self.debug_setpoint_xyz = self.create_publisher(Vector3, 'debug/setpoint_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_setpoint_rpy = self.create_publisher(Vector3, 'debug/setpoint_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))

    def callback(self, msg: Odometry):
                
        # Get state for optimizer
        state = get_state(msg)
        
        # Call the optimizer
        resp = self.solver.run(p=np.concatenate((
            state, 
            self.setpoint,
            mpc_state_weights,
            mpc_input_weights,
            mpc_final_weights
        )))

        try:
            u = np.array(resp.solution)[:6]
        except AttributeError:
            self.get_logger().error("No Solution")
            return

        cmd = Wrench()
        cmd.force.x = u[0] / force
        cmd.force.y = u[1] / force
        cmd.force.z = u[2] / force
        cmd.torque.x = u[3] / torque
        cmd.torque.y = u[4] / torque
        cmd.torque.z = u[5] / torque
        
        self.publisher.publish(cmd)
        
        # Publish debug messages
        if self.verbose > 1:
            
            tmp = Vector3()
            tmp.x = self.setpoint[0]
            tmp.y = self.setpoint[1]
            tmp.z = self.setpoint[2]
            self.debug_setpoint_xyz.publish(tmp)
            
            tmp = Vector3()
            euler = euler_from_quaternion(self.setpoint[6:10])
            tmp.x = euler[0] * 180 / np.pi
            tmp.y = euler[1] * 180 / np.pi
            tmp.z = euler[2] * 180 / np.pi
            self.debug_setpoint_rpy.publish(tmp)

    def set_setpoint(self, msg: Pose):
        self.setpoint = msg
        

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()