import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Wrench, TransformStamped, Vector3, Pose, Twist
from std_msgs.msg import Empty
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
from .parameters import force, torque
from .parameters import solo_tuning as tuning
from .pose_match_mpc_generator.parameters import nc, nu
from .tf_utils import quaternion_multiply, quaternion_inverse


mpc_final_weights = tuning['mpc_final_weights']
mpc_state_weights = tuning['mpc_state_weights']
mpc_input_weights = tuning['mpc_input_weights']


floor_ = lambda x, n=1e-3: x if abs(x) > n else 0.0
abs = lambda x: x if x > 0 else -x
sgn = lambda x: (1 if x > 0 else -1) if floor_(x, 1e-2) != 0 else 0


class MPCController2(Node):
    def __init__(self):
        super().__init__('mpc_advance')
        self.declare_parameter('verbose', 0)
        self.declare_parameter('nc', nc)
        self.declare_parameter('freq', 30.0)
        self.declare_parameter('ra_len', 9)
        self.declare_parameter('dock_dist', 0.35)
        self.declare_parameter('dock_vel', 0.1)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.nc = self.get_parameter('nc').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value
        self.ra_len = self.get_parameter('ra_len').get_parameter_value().integer_value
        self.dock_dist = self.get_parameter('dock_dist').get_parameter_value().double_value
        self.dock_vel = 1 - self.get_parameter('dock_vel').get_parameter_value().double_value * self.dt

        self.target_state = None
        self.prev_target_state = [None] * self.ra_len
        self.chaser_states = [None] * self.nc
        self.cmd = Wrench()
        self.cmd_flag = False
        self.approach_flag = [False] * self.nc

        # Generate offset
        self.offset = np.array([-1, 0, 0, 0, 0, 0, 1], dtype=np.float64)
        if self.nc > 1:
            self.offset = np.concatenate((self.offset, np.array([0, -1, 0, 0, 0, 0.7071, 0.7071], dtype=np.float64)))
        if self.nc > 2:
            self.offset = np.concatenate((self.offset, np.array([1, -1, 0, 0, 0, 0.7071, 0.7071], dtype=np.float64)))
        
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
        self.create_subscription(Empty, 'toggle_approach', self.approach_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        for i in range(self.nc):
            self.create_subscription(
                Empty, 
                'toggle_command', 
                partial(self.command_callback, i), 
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )

        # Publishers
        self.pubs = [self.create_publisher(
            Wrench, 
            'chaser_' + str(i) + '/thrust_cmd', 
            QoSPresetProfiles.get_from_short_key('system_default')
        ) for i in range(self.nc)]
        if self.verbose > 1:
            self.d_tp = self.create_publisher(Pose, '/debug/target_pose', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.d_tt = self.create_publisher(Twist, '/debug/target_twist', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.d_rp = [self.create_publisher(Pose, '/debug/relative_pose_{}'.format(i), QoSPresetProfiles.get_from_short_key('sensor_data')) for i in range(self.nc)]


        # Transform listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        # Callback to run controller
        self.create_timer(self.dt / 15, self.callback)

    def command_callback(self, msg):
        self.cmd_flag = not self.cmd_flag
        self.get_logger().info("Command flag set to {}".format(self.cmd_flag))

    def approach_callback(self, chaser_num, msg):
        self.approach_flag = not self.approach_flag
        self.get_logger().info("Approach flag set to {}".format(self.approach_flag))

    def get_odom(self, chaser_num, msg: Odometry):
        # self.get_logger().info("Got chaser {}".format(chaser_num))
        if self.chaser_states[chaser_num] is None:
            self.chaser_states[chaser_num] = get_state(msg)

    def callback(self):
        if self.target_state is None or any(map(lambda x: x is None, self.chaser_states)) or not self.cmd_flag: return

        # Call the optimizer
        p = np.concatenate((
            np.concatenate(self.chaser_states),     # State of each chaser
            self.target_state,                      # Target state
            self.offset,                            # Offsets from target state for chasers
            mpc_state_weights,                      # State weights
            mpc_input_weights,                      # Input weights
            mpc_final_weights                       # Final weights
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
        
        if self.verbose > 1:
            tmp = Pose()
            tmp.position.x = self.target_state[0]
            tmp.position.y = self.target_state[1]
            tmp.position.z = self.target_state[2]
            tmp.orientation.x = self.target_state[6]
            tmp.orientation.y = self.target_state[7]
            tmp.orientation.z = self.target_state[8]
            tmp.orientation.w = self.target_state[9]
            self.d_tp.publish(tmp)

            tmp = Twist()
            tmp.linear.x = self.target_state[3]
            tmp.linear.y = self.target_state[4]
            tmp.linear.z = self.target_state[5]
            tmp.angular.x = self.target_state[10]
            tmp.angular.y = self.target_state[11]
            tmp.angular.z = self.target_state[12]
            self.d_tt.publish(tmp)

            for i in range(self.nc):
                tmp = Pose()
                tmp.position.x = self.chaser_states[i][0] - self.target_state[0]
                tmp.position.y = self.chaser_states[i][1] - self.target_state[1]
                tmp.position.z = self.chaser_states[i][2] - self.target_state[2]
                tmp1 = quaternion_multiply(self.chaser_states[i][6:10], quaternion_inverse(self.target_state[6:10]))
                tmp.orientation.x = tmp1[0]
                tmp.orientation.y = tmp1[1]
                tmp.orientation.z = tmp1[2]
                tmp.orientation.w = tmp1[3]
                self.d_rp[i].publish(tmp)

        if self.approach_flag:
            for i in range(self.nc):
                self.offset[i*7 + 0] = sgn(self.offset[i*7 + 0]) * max(abs(self.offset[i*7 + 0] * self.dock_vel), self.dock_dist)
                self.offset[i*7 + 1] = sgn(self.offset[i*7 + 1]) * max(abs(self.offset[i*7 + 1] * self.dock_vel), self.dock_dist)
                self.offset[i*7 + 2] = sgn(self.offset[i*7 + 2]) * max(abs(self.offset[i*7 + 2] * self.dock_vel), self.dock_dist)

            self.get_logger().info("Approaching: {}".format(self.offset))

        self.target_state = None
        self.chaser_states = [None] * self.nc

    def get_target_state(self, msg: TransformStamped):
        # self.get_logger().info("Got target")
        if any(map(lambda x: x is None, self.prev_target_state)): 
            tmp = np.array([
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
            tmp[6:10] /= np.linalg.norm(tmp[6:10])
            self.prev_target_state = self.prev_target_state[1:] + [tmp]
        else:

            # Calculate velocity
            v = [
                (msg.transform.translation.x - self.prev_target_state[-1][0]) / self.dt,
                (msg.transform.translation.y - self.prev_target_state[-1][1]) / self.dt,
                (msg.transform.translation.z - self.prev_target_state[-1][2]) / self.dt,
            ]

            # Calculate omega
            eul = np.array(euler_from_quaternion([
                msg.transform.rotation.x,
                msg.transform.rotation.y, 
                msg.transform.rotation.z, 
                msg.transform.rotation.w])) - np.array(euler_from_quaternion(self.prev_target_state[-1][6:10]))
            omega = eul / self.dt

            # Create state vector
            tmp = np.array([
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
                floor_(v[0], 0.1),
                floor_(v[1], 0.1),
                floor_(v[2], 0.1),
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
                floor_(omega[0], 0.02),
                floor_(omega[1], 0.02),
                floor_(omega[2], 0.02),
            ], dtype=np.float64)
            tmp[6:10] /= np.linalg.norm(tmp[6:10])

            # Don't update on huge changes
            for j in [3, 4, 5, 10, 11, 12]:
                tmp[j] = tmp[j] if abs(tmp[j] - self.prev_target_state[-1][j]) < 10 else self.prev_target_state[-1][j]

            self.prev_target_state = self.prev_target_state[1:] + [tmp]
            for j in [3, 4, 5, 10, 11, 12]:
                tmp[j]  = floor_(sum([self.prev_target_state[i][j] for i in range(self.ra_len)]) / self.ra_len, 0.02)

        self.target_state = tmp


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