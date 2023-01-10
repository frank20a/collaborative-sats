import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import *
from copy import deepcopy

import numpy as np

from .tf_utils import *


def sde_solver(a, b, c):
    if a == 0 or b == c == 0:
        return None, None
    d = b**2 - 4*a*c 
    if d < 0:
        return None, None
    elif d < 1e-5:
        return -b / (2*a), None
    else:
        return (-b + np.sqrt(d)) / (2*a), (-b - np.sqrt(d)) / (2*a)

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
        self.target_prev_estim = None

        # Subscribers(s)
        self.create_subscription(
            PoseStamped,
            'pose',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
    def target_odom_callback(self, msg: Odometry):
        self.target_curr_tf = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
        ])

        if self.target_init_tf is None:
            self.target_init_tf = self.target_curr_tf
        
    def callback(self, msg: PoseStamped):
        if self.target_init_tf is None:
            return

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

            self.get_logger().info(f'\nA= {est[0]**2 + est[1]**2 + est[2]**2}\nB= {2 * (self.target_init_tf[0] * est[0] + self.target_init_tf[1] * est[1] + self.target_init_tf[2] * est[2])}\nC= {self.target_init_tf[0]**2 + self.target_init_tf[1]**2 + self.target_init_tf[2]**2 - D2}\n')
            self.get_logger().info(f'\nf1 = {f1}\nf2 = {f2}\n')

            if f1 is None or f2 is None:
                f = 0
            else:
                f = max(f1, f2)
                f = f1

        req = TransformStamped()
        req.header.stamp = msg.header.stamp
        req.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        req.child_frame_id = self.get_namespace().strip('/') + '/estimated_pose'
        
        req.transform.translation.x = self.target_init_tf[0] + est[0] * f
        req.transform.translation.y = self.target_init_tf[1] + est[1] * f
        req.transform.translation.z = self.target_init_tf[2] + est[2] * f
        req.transform.rotation.x = self.target_init_tf[3]
        req.transform.rotation.y = self.target_init_tf[4]
        req.transform.rotation.z = self.target_init_tf[5]
        req.transform.rotation.w = self.target_init_tf[6]

        self.broadcaster.sendTransform(req)

        req = TransformStamped()
        req.header.stamp = msg.header.stamp
        req.header.frame_id = self.get_namespace().strip('/') + '/camera_optical'
        req.child_frame_id = self.get_namespace().strip('/') + '/raw_estimation'
        
        req.transform.translation.x = self.target_init_tf[0] + est[0]
        req.transform.translation.y = self.target_init_tf[1] + est[1]
        req.transform.translation.z = self.target_init_tf[2] + est[2]
        req.transform.rotation.x = self.target_init_tf[3]
        req.transform.rotation.y = self.target_init_tf[4]
        req.transform.rotation.z = self.target_init_tf[5]
        req.transform.rotation.w = self.target_init_tf[6]

        self.broadcaster.sendTransform(req)

        self.target_curr_tf = None


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
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        
        if node.target_prev_estim is None:
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
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()