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
import os, sys

from .flags import *
from .parameters import force, torque


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc')
        self.declare_parameter('verbose', 0)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        sys.path.insert(1, os.path.join(get_package_share_directory('control'), 'python_build/chaser_mpc'))
        import chaser_mpc
        
        self.solver = chaser_mpc.solver()            
        
        self.setpoint = Pose()
        self.setpoint.position.x = -1.0
        self.setpoint.position.y = -1.0
        self.setpoint.position.z = 1.0
        q = quaternion_from_euler(0.0, 0.0, 0.0)
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
        
        
        # self.publisher = self.create_publisher(Int16, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        self.publisher = self.create_publisher(Wrench, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        
        
        if self.verbose > 1:
            self.debug_setpoint_xyz = self.create_publisher(Vector3, 'debug/setpoint_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_setpoint_rpy = self.create_publisher(Vector3, 'debug/setpoint_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_cmd_xyz = self.create_publisher(Vector3, 'debug/cmd_xyz', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.debug_cmd_rpy = self.create_publisher(Vector3, 'debug/cmd_rpy', QoSPresetProfiles.get_from_short_key('sensor_data'))

    def callback(self, msg: Odometry):
                
        # Get state for optimizer
        state = get_state(self.setpoint, msg)
        
        # Call the optimizer
        resp = self.solver.run(p=state)
        # self.get_logger().info('Response: {}'.format(resp.solution[:6]))
        try:
            u = np.array(resp.solution)[:6]
        except AttributeError:
            self.get_logger().error("No Solution")
            return
        # self.get_logger().info(str(u))
        # return
        
        # Rotate the thrust vector to remove body frame orientation
        u[0:3] = vector_rotate_quaternion(u[0:3], quaternion_inverse(odometry2array(msg)[3:]))
        
        # cmd = Int16()
        # cmd.data = 0
        # for i, f in enumerate(u):
        #     # Skip roll and pitch control
        #     if i == 3 or i == 4: continue
            
        #     if f != 0:
        #         cmd.data |= flags[2 * i + (1 if f < 0 else 0)]
        
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