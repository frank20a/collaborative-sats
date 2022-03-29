import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np


class CombineEstimations(Node):
    def __init__(self):
        super().__init__('combine_estimations')
        self.broadcaster = TransformBroadcaster(self)
        
        self.declare_parameter('num_chasers', 2)
        self.declare_parameter('fps', 50)
        self.declare_parameter('verbose', 0)
        self.declare_parameter('duration', False)
        
        self.num_chasers = self.get_parameter('num_chasers').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.fps_flag = self.get_parameter('duration').get_parameter_value().bool_value
        
        # create a timer to publish the combined transform
        self.timer = self.create_timer(1.0/self.fps, self.publish_combined_transform)
        
        # create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        # for i in range(self.num_chasers):
        #     if not self.buffer.wait_for_transform_async('world', 'chaser_{}/estimated_pose'.format(i), Time(), Time(3)):
        #         self.get_logger().error("Could not get transform from world to chaser_{}/estimated_pose".format(i))
        #         self.destroy_node()
        #         raise Exception("Could not get transform from world to chaser_{}/estimated_pose".format(i))
        
    def publish_combined_transform(self):
        t = self.get_clock().now()
        
        try:
            # create a combined transform
            combined_transform = TransformStamped()
            combined_transform.header.stamp = self.get_clock().now().to_msg()
            combined_transform.header.frame_id = 'world'
            combined_transform.child_frame_id = 'estimated_pose'
            
            euler_avg = np.array([0, 0, 0], dtype=np.float32)
            trans_avg = np.array([0, 0, 0], dtype=np.float32)
            
            now = Time()
            for i in range(self.num_chasers):
                transform = self.buffer.lookup_transform('world', 'chaser_{}/estimated_pose'.format(i), now)

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
                euler_avg += np.array(euler_from_quaternion(quat))
                
            trans_avg /= self.num_chasers
            euler_avg /= self.num_chasers
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
        except TransformException as e: 
            if self.verbose > 0:
                self.get_logger().error("Could not get transform from world to chaser_{}/estimated_pose".format(i))
            
        if self.fps_flag:
            self.get_logger().info('Combine duration: %.3f ms' % ((self.get_clock().now() - t).nanoseconds / 1e6))


def main(args=None):
    rclpy.init(args=args)
    node = CombineEstimations()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()