import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Wrench, Pose, Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from .tf_utils import get_state, vector_rotate_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import os, sys
from functools import partial

from .flags import *
from .parameters import force, torque
from .parameters import dueto_tuning as tuning
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
        self.declare_parameter('dock_dist', 0.4)
        self.declare_parameter('dock_vel', 0.07)
        self.declare_parameter('prefix', 'chaser')
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.nc = self.get_parameter('nc').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value
        self.ra_len = self.get_parameter('ra_len').get_parameter_value().integer_value
        self.dock_dist = self.get_parameter('dock_dist').get_parameter_value().double_value
        self.dock_vel = 1 - self.get_parameter('dock_vel').get_parameter_value().double_value * self.dt
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value

        self.target_pose = None
        self.target_twist = None
        self.prev_target_state = [None] * self.ra_len
        self.chaser_states = [None] * self.nc
        self.cmd = Wrench()
        self.cmd_flag = False
        self.approach_flag = [False] * self.nc

        # Generate offset
        self.offset = np.array([-1, 0, 0, 0, 0, 0, 1], dtype=np.float64)
        # self.offset = np.array([1, 0, 0, 0, 0, 1, 0], dtype=np.float64)
        if self.nc > 1:
            self.offset = np.concatenate((self.offset, np.array([0, -1, 0, 0, 0, 0.7071, 0.7071], dtype=np.float64)))
        if self.nc > 2:
            self.offset = np.concatenate((self.offset, np.array([1, -1, 0, 0, 0, 0.7071, 0.7071], dtype=np.float64)))
        
        # Solver
        if self.prefix == 'chaser':
            sys.path.insert(1, os.path.join(get_package_share_directory('control'), 'python_build/pose_match_mpc'))
            import pose_match_mpc as optimizer
        elif self.prefix == 'slider':
            sys.path.insert(1, os.path.join(get_package_share_directory('slider_experiment'), 'python_build/slider_mpc'))
            import slider_mpc as optimizer
        else:
            raise ValueError('Unknown prefix {}'.format(self.prefix))
        self.solver = optimizer.solver()

        # Subscribers
        for i in range(self.nc):
            self.create_subscription(
                Odometry,
                self.prefix + f'_{i}/odom',
                partial(self.odometry_callback, i),
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )
            self.create_subscription(
                Empty, 
                self.prefix + f'_{i}/toggle_approach', 
                partial(self.approach_callback, i), 
                QoSPresetProfiles.get_from_short_key('system_default')
            )
            self.create_subscription(
                Pose,
                self.prefix + f'_{i}/relative_setpoint',
                partial(self.setpoint_callback, i),
                QoSPresetProfiles.get_from_short_key('system_default')
            )
        self.create_subscription(Empty, 'toggle_control', self.control_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.create_subscription(Pose, 'target/estimated_pose', self.set_target_pose, QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.create_subscription(Twist, 'target/estimated_twist', self.set_target_twist, QoSPresetProfiles.get_from_short_key('sensor_data'))

        # Publishers
        self.pubs = [self.create_publisher(
            Wrench, 
            self.prefix + f'_{i}/thrust_cmd', 
            QoSPresetProfiles.get_from_short_key('system_default')
        ) for i in range(self.nc)]
        if self.verbose > 1:
            self.d_rp = [self.create_publisher(Pose, '/debug/relative_pose_{}'.format(i), QoSPresetProfiles.get_from_short_key('sensor_data')) for i in range(self.nc)]


        # Transform listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        # Callback to run controller
        self.create_timer(self.dt / 15, self.callback)

    def setpoint_callback(self, chaser_num: int, msg: Pose):
        self.offset[7*chaser_num: 7*(chaser_num+1)] = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=np.float64)

    def control_callback(self, msg: Empty):
        self.cmd_flag = not self.cmd_flag
        self.get_logger().info("Command flag set to {}".format(self.cmd_flag))

    def approach_callback(self, chaser_num: int, msg: Empty):
        self.approach_flag[chaser_num] = not self.approach_flag[chaser_num]
        self.get_logger().info("Approach flag {} set to {}".format(chaser_num, self.approach_flag))

    def odometry_callback(self, chaser_num: int, msg: Odometry):
        # self.get_logger().info("Got chaser {}".format(chaser_num))
        if self.chaser_states[chaser_num] is None:
            self.chaser_states[chaser_num] = get_state(msg)

    def set_target_pose(self, msg: Pose):
        self.target_pose = msg

    def set_target_twist(self, msg: Twist):
        self.target_twist = msg

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
        # self.get_logger().info(f"{target_state}")

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
            for i in range(self.nc):
                tmp = Pose()
                ttmp = vector_rotate_quaternion(self.chaser_states[i][0:3] - target_state[0:3], quaternion_inverse(target_state[6:10]))
                tmp.position.x = float(ttmp[0])
                tmp.position.y = float(ttmp[1])
                tmp.position.z = float(ttmp[2])
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


def main(args=None):
    rclpy.init(args=args)
    node = MPCController2()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()