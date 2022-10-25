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
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

from .tf_utils import do_transform_pose

class ORBSLAMFilter(Node):
    def __init__(self):
        super().__init__('orbslam_filter')
        
        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        self.broadcaster = TransformBroadcaster(self)

        self.offset = [0.0, 0.0, 2.0]

        # Positions
        self.target_tf = None
        self.chaser_tf = None
        self.estim_tf = TransformStamped()
        self.estim_tf.transform.translation.x = self.offset[0]
        self.estim_tf.transform.translation.y = self.offset[1]
        self.estim_tf.transform.translation.z = self.offset[2]
        # self.prev = Pose()
        # self.prev.position.x = self.offset[0]
        # self.prev.position.x = self.offset[1]
        # self.prev.position.x = self.offset[2]

        # Subscribers(s)
        self.create_subscription(
            PoseStamped,
            'pose',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.create_subscription(
            Odometry,
            '/target/odom',
            self.set_target_tf,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

    def set_target_tf(self, msg):
        self.target_tf = msg.pose.pose
        # self.get_logger().info(f'{self.target_tf}')
        
    def callback(self, msg: Pose):
        stamp = msg.header.stamp
        msg = msg.pose
        msg.position.x += self.offset[0]
        msg.position.y += self.offset[1]
        msg.position.z += self.offset[2]

        if self.chaser_tf is None or self.target_tf is None or self.estim_tf is None:      
            # self.get_logger().info("NOFIND ch:{} ta:{} es:{}".format(not self.chaser_tf is None, not self.target_tf is None, not self.estim_tf is None))
            return
        else:

            tmp = do_transform_pose(msg, self.chaser_tf)

            factor = np.sqrt(
                (self.chaser_tf.transform.translation.x - self.target_tf.position.x)**2 + \
                (self.chaser_tf.transform.translation.y - self.target_tf.position.y)**2 + \
                (self.chaser_tf.transform.translation.z - self.target_tf.position.z)**2) / np.sqrt(
                (self.chaser_tf.transform.translation.x - tmp.position.x)**2 + \
                (self.chaser_tf.transform.translation.y - tmp.position.y)**2 + \
                (self.chaser_tf.transform.translation.z - tmp.position.z)**2)
            self.chaser_tf = None
            self.target_tf = None
            self.estim_tf = None

        # self.prev = msg
        self.get_logger().info(f'factor: {factor:1.3f}')

        req = TransformStamped()
        req.header.stamp = stamp
        req.header.frame_id = (self.get_namespace() + '/camera_optical').strip('/')
        req.child_frame_id = (self.get_namespace() + '/estimated_pose').strip('/')
        req.transform.translation.x = msg.position.x * factor
        req.transform.translation.y = msg.position.y * factor
        req.transform.translation.z = msg.position.z * factor

        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # eul = euler_from_quaternion(q)
        # q = quaternion_from_euler(eul[0] * factor, eul[1] * factor, eul[2] * factor)
        
        req.transform.rotation.x = q[0]
        req.transform.rotation.y = q[1]
        req.transform.rotation.z = q[2]
        req.transform.rotation.w = q[3]

        # self.get_logger().info('Publishing transform: {}'.format(req))

        self.broadcaster.sendTransform(req)
        

def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAMFilter()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.chaser_tf is None:
            try:
                node.chaser_tf = node.buffer.lookup_transform('world', (node.get_namespace() + '/camera_optical').lstrip('/'), Time(seconds=0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        
        if node.estim_tf is None:
            try:
                node.estim_tf = node.buffer.lookup_transform('world', (node.get_namespace() + '/estimated_pose').lstrip('/'), Time(seconds=0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()