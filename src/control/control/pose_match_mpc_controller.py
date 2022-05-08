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
from .parameters import force, torque, mpc_final_weights, mpc_input_weights, mpc_state_weights, dt
from .pose_match_mpc_generator.parameters import nc, nu


class MPCController2(Node):
    def __init__(self):
        super().__init__('mpc_advance')
        self.declare_parameter('verbose', 0)
        self.declare_parameter('nc', nc)
        self.declare_parameter('freq', 1/dt)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.nc = self.get_parameter('nc').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value

        self.target_state = None
        self.prev_target_state = None
        self.chaser_states = [None] * self.nc
        
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
        self.create_timer(self.dt / 5, self.callback)

    def get_odom(self, chaser_num, msg: Odometry):
        # self.get_logger().info("Got chaser {}".format(chaser_num))
        if self.chaser_states[chaser_num] is None:
            self.chaser_states[chaser_num] = get_state(msg)

    def callback(self):
        if self.target_state is None or any(map(lambda x: x is None, self.chaser_states)): return
        # self.get_logger().info("Running controller")

        # Call the optimizer
        resp = self.solver.run(p=np.concatenate((
            np.concatenate(self.chaser_states), 
            self.target_state,
            mpc_state_weights,
            mpc_input_weights,
            mpc_final_weights
        )))

        for i in range(self.nc):
            try:
                u = np.array(resp.solution)[nu * i: nu * (i + 1)]
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

            self.pubs[i].publish(cmd)
        
        self.target_state = None
        self.chaser_states = [None] * self.nc

    def get_target_state(self, msg: TransformStamped):
        # self.get_logger().info("Got target")

        if self.prev_target_state is None: 
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
        else:
            v = [
                (msg.transform.translation.x - self.prev_target_state[0]) / self.dt,
                (msg.transform.translation.y - self.prev_target_state[1]) / self.dt,
                (msg.transform.translation.z - self.prev_target_state[2]) / self.dt,
            ]

            eul = np.array(euler_from_quaternion([
                msg.transform.rotation.x,
                msg.transform.rotation.y, 
                msg.transform.rotation.z, 
                msg.transform.rotation.w])) - np.array(euler_from_quaternion(self.prev_target_state[6:10]))
            omega = eul / self.dt

            self.target_state = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
                v[0],
                v[1],
                v[2],
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
                omega[0],
                omega[1],
                omega[2]
            ], dtype=np.float64)
        
        self.prev_target_state = self.target_state

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