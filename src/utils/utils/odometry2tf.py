import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster

class Odometry2TF(Node):
    def __init__(self):
        super().__init__('odometry2tf')
        self.pose_br = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

    def callback(self, msg):
        t = TransformStamped()
        t.header = msg.header

        t.child_frame_id = msg.child_frame_id
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.pose_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry2TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()