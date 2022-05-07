import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

class SetpointPub(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')

        # Timer for setpoint tf publish
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(1 / 5.0, self.callback)
        self.setpoint_tf = TransformStamped()
        self.setpoint_tf.header.frame_id = (self.get_namespace() + '/estimated_pose').lstrip('/')
        self.setpoint_tf.child_frame_id = (self.get_namespace() + '/setpoint').lstrip('/')
        self.setpoint_tf.transform.translation.x = -1.0
        self.setpoint_tf.transform.translation.y = 0.0
        self.setpoint_tf.transform.translation.z = 0.0
        self.setpoint_tf.transform.rotation.x = 0.0
        self.setpoint_tf.transform.rotation.y = 0.0
        self.setpoint_tf.transform.rotation.z = 0.0
        self.setpoint_tf.transform.rotation.w = 1.0
        
    def callback(self):
        self.setpoint_tf.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.setpoint_tf)
        

def main(args=None):
    rclpy.init(args=args)
    node = SetpointPub()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()