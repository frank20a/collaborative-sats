import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Wrench, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion
from .tf_utils import get_state
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import os, sys
from functools import partial

from .flags import *
from .parameters import force, torque, mpc_final_weights, mpc_input_weights, mpc_state_weights
from .pose_match_mpc_generator.parameters import nc, nu


class MPCController2(Node):
    def __init__(self):
        super().__init__('mpc_advance')
        self.declare_parameter('verbose', 0)
        self.declare_parameter('nc', nc)
        self.declare_parameter('freq', 30.0)
        self.declare_parameter('ra_len', 5)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.nc = self.get_parameter('nc').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value
        self.ra_len = self.get_parameter('ra_len').get_parameter_value().integer_value

        self.target_state = None
        self.prev_target_state = [None] * self.ra_len
        self.chaser_states = [None] * self.nc
        self.cmd = Wrench()
        
        # Solver
        sys.path.insert(1, os.path.join(get_package_share_directory('control'), 'python_build/pose_match_mpc'))
        import pose_match_mpc
        self.solver = pose_match_mpc.solver()       

        # Subscribers
        for i in range(self.nc):
            self.create_subscription(
                Odometry,
                'chaser_' + str(i) + '/odom',
                partial(self.get_odom, i),
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )

        # Publishers
        self.pubs = [self.create_publisher(
            Wrench, 
            'chaser_' + str(i) + '/thrust_cmd', 
            QoSPresetProfiles.get_from_short_key('system_default')
        ) for i in range(self.nc)]

        # Transform listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        # Callback to run controller
        self.create_timer(self.dt / 15, self.callback)

    def get_odom(self, chaser_num, msg: Odometry):
        # self.get_logger().info("Got chaser {}".format(chaser_num))
        if self.chaser_states[chaser_num] is None:
            self.chaser_states[chaser_num] = get_state(msg)

    def callback(self):
        if self.target_state is None or any(map(lambda x: x is None, self.chaser_states)): return
        # self.get_logger().info("Running controller")

        # Call the optimizer
        p = np.concatenate((
            np.concatenate(self.chaser_states),                                 # State of each chaser
            self.target_state,                                                  # Target state
            np.array([-1, 0, 0, 0, 0, 0, 1], dtype=np.float64),                 # Offsets from target state for chaser 1
            np.array([0, -1, 0, 0, 0, 0.7071, 0.7071], dtype=np.float64),       # Offsets from target state for chaser 2
            mpc_state_weights,                                                  # State weights
            mpc_input_weights,                                                  # Input weights
            mpc_final_weights                                                   # Final weights
        ))
        resp = self.solver.run(p=p)

        for i in range(self.nc):
            try:
                u = np.array(resp.solution)[nu * i: nu * (i + 1)]
            except AttributeError:
                self.get_logger().error("No Solution")
                self.cmd.force.x = -self.cmd.force.x / 3
                self.cmd.force.y = -self.cmd.force.y / 3
                self.cmd.force.z = -self.cmd.force.z / 3
                self.cmd.torque.x = -self.cmd.torque.x / 3
                self.cmd.torque.y = -self.cmd.torque.y / 3
                self.cmd.torque.z = -self.cmd.torque.z / 3
            else:
                self.cmd.force.x = u[0] / force
                self.cmd.force.y = u[1] / force
                self.cmd.force.z = u[2] / force
                self.cmd.torque.x = u[3] / torque
                self.cmd.torque.y = u[4] / torque
                self.cmd.torque.z = u[5] / torque
            finally:
                self.pubs[i].publish(self.cmd)
        
        self.target_state = None
        self.chaser_states = [None] * self.nc

    def get_target_state(self, msg: TransformStamped):
        # self.get_logger().info("Got target")
        if any(map(lambda x: x is None, self.prev_target_state)): 
            self.target_state = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
                0,
                0,
                0,
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
                0,
                0,
                0
            ], dtype=np.float64)
            self.prev_target_state[6:10] /= np.linalg.norm(self.prev_target_state[6:10])
            self.prev_target_state = self.prev_target_state[1:] + [self.target_state]
        else:
            v = [
                (msg.transform.translation.x - self.prev_target_state[-1][0]) / self.dt,
                (msg.transform.translation.y - self.prev_target_state[-1][1]) / self.dt,
                (msg.transform.translation.z - self.prev_target_state[-1][2]) / self.dt,
            ]

            eul = np.array(euler_from_quaternion([
                msg.transform.rotation.x,
                msg.transform.rotation.y, 
                msg.transform.rotation.z, 
                msg.transform.rotation.w])) - np.array(euler_from_quaternion(self.prev_target_state[-1][6:10]))
            omega = eul / self.dt

            self.target_state = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
                v[0] if abs(v[0]) > 0.1 else 0,
                v[1] if abs(v[1]) > 0.1 else 0,
                v[2] if abs(v[2]) > 0.1 else 0,
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
                omega[0] if abs(omega[0]) > 0.1 else 0,
                omega[1] if abs(omega[1]) > 0.1 else 0,
                omega[2] if abs(omega[2]) > 0.1 else 0
            ], dtype=np.float64)
            self.prev_target_state[6:10] /= np.linalg.norm(self.prev_target_state[6:10])
            # self.get_logger().info("Target state: {}".format(self.target_state))

            self.prev_target_state = self.prev_target_state[1:] + [self.target_state]
            self.target_state[3]  = sum([self.prev_target_state[i][3] for i in range(self.ra_len)]) / self.ra_len
            self.target_state[4]  = sum([self.prev_target_state[i][4] for i in range(self.ra_len)]) / self.ra_len
            self.target_state[5]  = sum([self.prev_target_state[i][5] for i in range(self.ra_len)]) / self.ra_len
            self.target_state[10] = sum([self.prev_target_state[i][10] for i in range(self.ra_len)]) / self.ra_len
            self.target_state[11] = sum([self.prev_target_state[i][11] for i in range(self.ra_len)]) / self.ra_len
            self.target_state[12] = sum([self.prev_target_state[i][12] for i in range(self.ra_len)]) / self.ra_len

def main(args=None):
    rclpy.init(args=args)
    node = MPCController2()

    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.target_state is None:
            try:
                node.get_target_state(node.buffer.lookup_transform('world', 'estimated_pose', Time(seconds=0)))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass

    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()