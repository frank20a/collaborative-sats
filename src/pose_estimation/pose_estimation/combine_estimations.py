import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np


class CombineEstimations(Node):
    def __init__(self):
        super().__init__('combine_estimations')
        self.broadcaster = TransformBroadcaster(self)
        
        self.declare_parameter('num_chasers', 2)
        self.declare_parameter('freq', 30.0)
        self.declare_parameter('verbose', 0)
        
        self.num_chasers = self.get_parameter('num_chasers').get_parameter_value().integer_value
        self.freq = self.get_parameter('freq').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        # create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        self.tfs = [None for i in range(self.num_chasers)]

        self.create_timer(1 / self.freq, self.combine_n_publish)
        
    def combine_n_publish(self):
        # create a combined transform
        combined_transform = TransformStamped()
        combined_transform.header.stamp = self.get_clock().now().to_msg()
        combined_transform.header.frame_id = 'world'
        combined_transform.child_frame_id = 'estimated_pose'
        
        euler_avg_num = np.array([0, 0, 0], dtype=np.float32)
        euler_avg_denum = np.array([0, 0, 0], dtype=np.float32)
        trans_avg = np.array([0, 0, 0], dtype=np.float32)
        
        c = 0
        for i in range(self.num_chasers):
            if self.tfs[i] is None: continue

            c += 1
            transform = self.tfs[i]

            trans_avg += np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ], dtype=np.float32)
            
            quat = [
                transform.transform.rotation.x, 
                transform.transform.rotation.y, 
                transform.transform.rotation.z, 
                transform.transform.rotation.w
            ]
            euler_avg_num += np.sin(np.array(euler_from_quaternion(quat)))
            euler_avg_denum += np.cos(np.array(euler_from_quaternion(quat)))

            self.tfs[i] = None

        if c == 0: return
            
        trans_avg /= c
        euler_avg = np.arctan2(euler_avg_num, euler_avg_denum)
        quat_avg = quaternion_from_euler(euler_avg[0], euler_avg[1], euler_avg[2])
        
        combined_transform.transform.translation.x = float(trans_avg[0])
        combined_transform.transform.translation.y = float(trans_avg[1])
        combined_transform.transform.translation.z = float(trans_avg[2])
        
        combined_transform.transform.rotation.x = float(quat_avg[0])
        combined_transform.transform.rotation.y = float(quat_avg[1])
        combined_transform.transform.rotation.z = float(quat_avg[2])
        combined_transform.transform.rotation.w = float(quat_avg[3])
        
            
        # publish the combined transform
        if self.verbose > 0: self.get_logger().info('Publishing combined transform')
        self.broadcaster.sendTransform(combined_transform)


def main(args=None):
    rclpy.init(args=args)
    node = CombineEstimations()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        for i in range(node.num_chasers):
            if node.tfs[i] is not None: continue
            
            try:
                node.tfs[i] = node.buffer.lookup_transform('world', 'chaser_{}/estimated_pose'.format(i), Time(seconds=0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
            
        # if all(map(lambda x: x is not None, node.tfs)):
        #     node.combine_n_publish()
        #     node.tfs = [None for i in range(node.num_chasers)]
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()