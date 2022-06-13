import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster

from tf_transformations import euler_from_quaternion
import numpy as np

def lim(x, n=5):
    if -n < x < n: return x
    return 0.0

class ViconFilter(Node):
    def __init__(self):
        super().__init__('vicon_filter')

        self.pose_br = TransformBroadcaster(self)

        self.offset = None
        self.offset_flag = False
        self.go = False

        self.prev_state = None
        self.prev_time = None
        
        self.create_subscription(Pose, 'input', self.callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        # self.create_timer(1/50, self.dummy)
        self.create_subscription(Empty, 'set_offset', self.set_offset, QoSPresetProfiles.get_from_short_key('system_default'))
        self.create_subscription(Empty, 'enable_vicon', self.set_go, QoSPresetProfiles.get_from_short_key('system_default'))

        self.pub = self.create_publisher(Odometry, 'slider_0/odom', QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    def callback(self, msg: Pose):
        if not self.offset_flag:
            self.offset = [msg.position.x, msg.position.y, msg.position.z]
            return
        
        if self.go:
            tmp = TransformStamped()
            tmp.header.frame_id = 'world'
            tmp.header.stamp = self.get_clock().now().to_msg()
            tmp.child_frame_id = 'slider_0/body'
            tmp.transform.translation.x = msg.position.x - self.offset[0]
            tmp.transform.translation.y = msg.position.y - self.offset[1]
            tmp.transform.translation.z = msg.position.z - self.offset[2]
            tmp.transform.rotation.x = msg.orientation.x 
            tmp.transform.rotation.y = msg.orientation.y
            tmp.transform.rotation.z = msg.orientation.z
            tmp.transform.rotation.w = msg.orientation.w
            self.pose_br.sendTransform(tmp)

            tmp = Pose()
            tmp.position.x = msg.position.x - self.offset[0]
            tmp.position.y = msg.position.y - self.offset[1]
            tmp.position.z = msg.position.z - self.offset[2]
            tmp.orientation = msg.orientation
            tmp1 = Twist()
            if self.prev_state != None:
                tmp1.linear.x = lim(1e9 * (tmp.position.x - self.prev_state.position.x) / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.linear.y = lim(1e9 * (tmp.position.y - self.prev_state.position.y) / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.linear.z = lim(1e9 * (tmp.position.z - self.prev_state.position.z) / (self.get_clock().now() - self.prev_time).nanoseconds)

                diff = np.array(euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))) - \
                    np.array(euler_from_quaternion((self.prev_state.orientation.x, self.prev_state.orientation.y, self.prev_state.orientation.z, self.prev_state.orientation.w)))
                tmp1.angular.x = lim(1e9 * diff[0] / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.angular.y = lim(1e9 * diff[1] / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.angular.z = lim(1e9 * diff[2] / (self.get_clock().now() - self.prev_time).nanoseconds)
            self.prev_state = tmp
            self.prev_time = self.get_clock().now()

            res = Odometry()
            res.pose.pose = tmp
            res.twist.twist = tmp1
            self.pub.publish(res)

    def dummy(self):
        if not self.offset_flag:
            self.offset = [0.0, 0.0, 0.0]
            return
        
        if self.go:
            tmp = TransformStamped()
            tmp.header.frame_id = 'world'
            tmp.header.stamp = self.get_clock().now().to_msg()
            tmp.child_frame_id = 'slider_0/body'
            tmp.transform.translation.x = 0.2
            tmp.transform.translation.y = 0.2
            tmp.transform.translation.z = 0.0
            tmp.transform.rotation.x = 0.0
            tmp.transform.rotation.y = 0.0
            tmp.transform.rotation.z = 0.0
            tmp.transform.rotation.w = 1.0
            self.pose_br.sendTransform(tmp)

            tmp = Pose()
            tmp.position.x = 0.2
            tmp.position.y = 0.2
            tmp.position.z = 0.0
            tmp.orientation.x = 0.0
            tmp.orientation.y = 0.0
            tmp.orientation.z = 0.0
            tmp.orientation.w = 1.0
            tmp1 = Twist()
            if self.prev_state != None:
                tmp1.linear.x = lim(1e9 * (tmp.position.x - self.prev_state.position.x) / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.linear.y = lim(1e9 * (tmp.position.y - self.prev_state.position.y) / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.linear.z = lim(1e9 * (tmp.position.z - self.prev_state.position.z) / (self.get_clock().now() - self.prev_time).nanoseconds)

                diff = np.array(euler_from_quaternion((0.0, 0.0, 0.0, 1.0))) - \
                    np.array(euler_from_quaternion((self.prev_state.orientation.x, self.prev_state.orientation.y, self.prev_state.orientation.z, self.prev_state.orientation.w)))
                tmp1.angular.x = lim(1e9 * diff[0] / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.angular.y = lim(1e9 * diff[1] / (self.get_clock().now() - self.prev_time).nanoseconds)
                tmp1.angular.z = lim(1e9 * diff[2] / (self.get_clock().now() - self.prev_time).nanoseconds)
            self.prev_state = tmp
            self.prev_time = self.get_clock().now()

            res = Odometry()
            res.pose.pose = tmp
            res.twist.twist = tmp1
            self.pub.publish(res)

    def set_offset(self, _):
        self.get_logger().info("Offset set to {}".format(self.offset))
        self.offset_flag = True

    def set_go(self, _):
        if self.offset_flag:
            self.get_logger().info("Started Publishing")
            self.go = True
        else:
            self.get_logger().error("Offset flag not set... Ignoring")
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ViconFilter()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()