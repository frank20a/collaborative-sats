import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSPresetProfiles
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import *
from copy import deepcopy

import numpy as np

from .tf_utils import *


def abs_min(a, b):
    if abs(a) < abs(b):
        return a
    else:
        return b

def abs_min_conditional(a, b, c):
    if abs(a) < abs(b):
        if abs(a) >= c:
            return a
        else:
            return b
    else:
        if abs(b) >= c:
            return b
        else:
            return a


def sde_solver(a, b, c):
    if a == 0:
        if b == 0:
            return None, None
        else:
            return -c / b, -c / b

    d = b**2 - 4*a*c

    return (-b + np.emath.sqrt(d)) / (2*a), (-b - np.emath.sqrt(d)) / (2*a)


def decide_tf(tf1, tf2):
    theta1 = np.arccos(tf1.transform.translation.z / np.linalg.norm([tf1.transform.translation.x, tf1.transform.translation.y, tf1.transform.translation.z]))
    theta2 = np.arccos(tf2.transform.translation.z / np.linalg.norm([tf2.transform.translation.x, tf2.transform.translation.y, tf2.transform.translation.z]))

    if theta1 < theta2:
        return tf1
    return tf2


class ORBSLAMFilter(Node):
    def __init__(self):
        super().__init__('orbslam_filter')
        
        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        self.broadcaster = TransformBroadcaster(self)


        # Positions
        self.target_init_tf = None
        self.target_curr_tf = None
        self.estim_curr_tf = None

        self.target_pose = None
        self.chaser_pose = None

        # Factors
        self.fs = [1, 1]

        # Subscribers(s)
        self.create_subscription(
            PoseStamped,
            'pose',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.verbose = self.create_publisher(
            String,
            'orbslam_verbose',
            QoSPresetProfiles.get_from_short_key('system_default')
        )

        self.create_subscription(Odometry, 'odom', self.chaser_odom_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.create_subscription(Odometry, '/target/odom', self.target_odom_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))

    def chaser_odom_callback(self, msg: Odometry):
        self.chaser_pose = msg.pose.pose

    def decide_tf(self, tf1: TransformStamped, tf2: TransformStamped):
        d1 = np.linalg.norm([
            tf1.transform.translation.x - self.target_curr_tf[0],
            tf1.transform.translation.y - self.target_curr_tf[1], 
            tf1.transform.translation.z - self.target_curr_tf[2]
        ])
        d2 = np.linalg.norm([
            tf2.transform.translation.x - self.target_curr_tf[0],
            tf2.transform.translation.y - self.target_curr_tf[1], 
            tf2.transform.translation.z - self.target_curr_tf[2]
        ])

        if d1 < d2:
            return tf1
        return tf2

    def target_odom_callback(self, msg: Odometry):
        self.target_pose = msg.pose.pose

    def do_tf_calculations(self, est, f):
        tf = TransformStamped()

        # Calculate e-movement
        trans = self.target_init_tf[:3] + est[:3] * f
        trans = vector_rotate_quaternion(trans, quaternion_conjugate(quaternion_multiply(self.target_init_tf[3:], quaternion_inverse(self.estim_curr_tf[3:]))))
        q = quaternion_multiply(est[3:], self.target_init_tf[3:])
        q = quaternion_multiply(q, [0, 0, 0.7071068, 0.7071068])

        # Set the tf
        tf.transform.translation.x = float(trans[0])
        tf.transform.translation.y = float(trans[1])
        tf.transform.translation.z = float(trans[2])
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        return tf

    def callback(self, msg: PoseStamped):
        if self.target_init_tf is None or self.estim_curr_tf is None:
            return

        verb = ''

        est = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        if self.target_curr_tf is not None:
            D2 = self.target_curr_tf[0]**2 + self.target_curr_tf[1]**2 + self.target_curr_tf[2]**2
            
            f1, f2 = sde_solver(
                est[0]**2 + est[1]**2 + est[2]**2,
                2 * (self.target_init_tf[0] * est[0] + self.target_init_tf[1] * est[1] + self.target_init_tf[2] * est[2]),
                self.target_init_tf[0]**2 + self.target_init_tf[1]**2 + self.target_init_tf[2]**2 - D2
            )

            verb += f'\nA= {est[0]**2 + est[1]**2 + est[2]**2:.3f}\nB= {2 * (self.target_init_tf[0] * est[0] + self.target_init_tf[1] * est[1] + self.target_init_tf[2] * est[2]):.3f}\nC= {self.target_init_tf[0]**2 + self.target_init_tf[1]**2 + self.target_init_tf[2]**2 - D2:.3f}\n'
            if f1 is not None and f2 is not None:
                if np.iscomplex(f1) or np.iscomplex(f2):
                    # f1 = np.abs(f1)
                    # f2 = np.abs(f2) * -1
                    f1 = f1.real 
                    f2 = f2.real * -1
                self.fs = [f1, f2]

            try:
                verb += f'\nf1 = {f1:.3f}\nf2 = {f2:.3f}\n'
            except TypeError:
                verb += f'\nf1 = {f1}\nf2 = {f2}\n'

        eul = None
        if self.target_pose is not None and self.chaser_pose is not None:
            q_ = quaternion_from_euler(
                0.0,
                np.arctan2(- self.target_pose.position.z + self.chaser_pose.position.z, np.sqrt((self.target_pose.position.x - self.chaser_pose.position.x)**2 + (self.target_pose.position.y - self.chaser_pose.position.y)**2)),
                np.arctan2(self.target_pose.position.y - self.chaser_pose.position.y, self.target_pose.position.x - self.chaser_pose.position.x),
            )
            qq_ = [
                self.chaser_pose.orientation.x,
                self.chaser_pose.orientation.y,
                self.chaser_pose.orientation.z,
                self.chaser_pose.orientation.w
            ]
            eul = euler_from_quaternion(quaternion_multiply(q_, quaternion_conjugate(qq_)))

        tf1 = self.do_tf_calculations(est, self.fs[0])
        tf1.header.stamp = msg.header.stamp
        tf1.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        tf1.child_frame_id = self.get_namespace().strip('/') + '/estimated_pose_1'

        
        tf2 = self.do_tf_calculations(est, self.fs[1])
        tf2.header.stamp = msg.header.stamp
        tf2.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        tf2.child_frame_id = self.get_namespace().strip('/') + '/estimated_pose_2'

        req = self.decide_tf(tf1, tf2)
        req.header.stamp = msg.header.stamp
        req.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        req.child_frame_id = self.get_namespace().strip('/') + '/estimated_pose'
        if eul is not None and -0.65 < eul[0] < 0.65 and -0.65 < eul[1] < 0.65 and -0.65 < eul[2] < 0.65:
            self.broadcaster.sendTransform(tf1)
            self.broadcaster.sendTransform(tf2)
            self.broadcaster.sendTransform(req)
        
        # Publish the raw estimation with no scaling
        # req = self.do_tf_calculations(est, 1)
        # req.header.stamp = msg.header.stamp
        # req.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        # req.child_frame_id = self.get_namespace().strip('/') + '/raw_estimation'
        # self.broadcaster.sendTransform(req)

        self.target_curr_tf = None
        
        self.verbose.publish(String(data=verb))


def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAMFilter()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.target_curr_tf is None:
            try:
                tmp = node.buffer.lookup_transform((node.get_namespace() + '/camera_optical').lstrip('/'), 'target/body', Time(seconds=0))
                node.target_curr_tf = np.array([
                        tmp.transform.translation.x,
                        tmp.transform.translation.y,
                        tmp.transform.translation.z,
                        tmp.transform.rotation.x,
                        tmp.transform.rotation.y,
                        tmp.transform.rotation.z,
                        tmp.transform.rotation.w
                ])

                if node.target_init_tf is None:
                    node.target_init_tf = node.target_curr_tf
                if node.estim_curr_tf is None:
                    node.estim_curr_tf = node.target_curr_tf
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        
        if node.estim_curr_tf is None:
            try:
                tmp = node.buffer.lookup_transform((node.get_namespace() + '/camera_optical').lstrip('/'), 'target_body', Time(seconds=0))
                node.estim_curr_tf = np.array([
                        tmp.transform.translation.x,
                        tmp.transform.translation.y,
                        tmp.transform.translation.z,
                        tmp.transform.rotation.x,
                        tmp.transform.rotation.y,
                        tmp.transform.rotation.z,
                        tmp.transform.rotation.w
                ])
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()