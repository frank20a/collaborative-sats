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
        self.declare_parameter('dock_dist', 0.3)
        self.declare_parameter('dock_vel', 0.07)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.nc = self.get_parameter('nc').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value
        self.ra_len = self.get_parameter('ra_len').get_parameter_value().integer_value
        self.dock_dist = self.get_parameter('dock_dist').get_parameter_value().double_value
        self.dock_vel = 1 - self.get_parameter('dock_vel').get_parameter_value().double_value * self.dt

        self.target_pose = None
        self.target_twist = None
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
        self.create_subscription(Empty, 'toggle_command', self.command_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        for i in range(self.nc):
            self.create_subscription(
                Empty, 
                f'chaser_{i}/toggle_approach', 
                partial(self.approach_callback, i), 
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )
        self.create_subscription(Pose, 'target/estimated_pose', self.set_target_pose, QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.create_subscription(Twist, 'target/estimated_twist', self.set_target_twist, QoSPresetProfiles.get_from_short_key('sensor_data'))

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
        self.approach_flag[chaser_num] = not self.approach_flag[chaser_num]
        self.get_logger().info("Approach flag {} set to {}".format(chaser_num, self.approach_flag))

    def get_odom(self, chaser_num, msg: Odometry):
        # self.get_logger().info("Got chaser {}".format(chaser_num))
        if self.chaser_states[chaser_num] is None:
            self.chaser_states[chaser_num] = get_state(msg)

    def callback(self):
        if self.target_pose is None or self.target_twist is None or any(map(lambda x: x is None, self.chaser_states)) or not self.cmd_flag: return
        target_state = np.array([
            self.target_pose.position.x,
            self.target_pose.position.y,
            self.target_pose.position.z,
            self.target_twist.linear.x,
            self.target_twist.linear.y,
            self.target_twist.linear.z,
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w,
            self.target_twist.angular.x,
            self.target_twist.angular.y,
            self.target_twist.angular.z
        ], dtype=np.float64)
        self.get_logger().info(f"{target_state}")

        # Call the optimizer
        p = np.concatenate((
            np.concatenate(self.chaser_states),     # State of each chaser
            target_state,                      # Target state
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
            tmp.position.x = target_state[0]
            tmp.position.y = target_state[1]
            tmp.position.z = target_state[2]
            tmp.orientation.x = target_state[6]
            tmp.orientation.y = target_state[7]
            tmp.orientation.z = target_state[8]
            tmp.orientation.w = target_state[9]
            self.d_tp.publish(tmp)

            tmp = Twist()
            tmp.linear.x = target_state[3]
            tmp.linear.y = target_state[4]
            tmp.linear.z = target_state[5]
            tmp.angular.x = target_state[10]
            tmp.angular.y = target_state[11]
            tmp.angular.z = target_state[12]
            self.d_tt.publish(tmp)

            for i in range(self.nc):
                tmp = Pose()
                tmp.position.x = self.chaser_states[i][0] - target_state[0]
                tmp.position.y = self.chaser_states[i][1] - target_state[1]
                tmp.position.z = self.chaser_states[i][2] - target_state[2]
                tmp1 = quaternion_multiply(self.chaser_states[i][6:10], quaternion_inverse(target_state[6:10]))
                tmp.orientation.x = tmp1[0]
                tmp.orientation.y = tmp1[1]
                tmp.orientation.z = tmp1[2]
                tmp.orientation.w = tmp1[3]
                self.d_rp[i].publish(tmp)

        for i in range(self.nc):
            if self.approach_flag[i]:
                self.offset[i*7 + 0] = sgn(self.offset[i*7 + 0]) * max(abs(self.offset[i*7 + 0] * self.dock_vel), self.dock_dist)
                self.offset[i*7 + 1] = sgn(self.offset[i*7 + 1]) * max(abs(self.offset[i*7 + 1] * self.dock_vel), self.dock_dist)
                self.offset[i*7 + 2] = sgn(self.offset[i*7 + 2]) * max(abs(self.offset[i*7 + 2] * self.dock_vel), self.dock_dist)

            # self.get_logger().info("Approaching {}: {}".format(i, self.offset))

        self.target_pose = None
        self.target_twist = None
        self.chaser_states = [None] * self.nc

    def set_target_pose(self, msg: Pose):
        self.target_pose = msg

    def set_target_twist(self, msg: Twist):
        self.target_twist = msg


def main(args=None):
    rclpy.init(args=args)
    node = MPCController2()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()