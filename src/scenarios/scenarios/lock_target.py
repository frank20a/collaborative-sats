import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles

from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from tf2_ros.buffer import Buffer
# from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc')
        
        # Create a tf2 buffer and listener
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        
        # Node parameters
        self.declare_parameter('verbose', 0)
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        # Subscriptions
        self.create_subscription(
            Pose,
            'setpoint',
            self.set_setpoint,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        # Publishers
        # self.publisher = self.create_publisher(Int16, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        self.publisher = self.create_publisher(Wrench, 'thrust_cmd', QoSPresetProfiles.get_from_short_key('system_default'))
        
        self.target_pose = None
        self.chaser_pose = None
        
    def callback(self):
        if self.target_pose is None or self.chaser_pose is None: return
        
        # Code
        self.target_pose = self.chaser_pose = None
        

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.announcement is None: continue
        
        try:
            node.target_pose = node.buffer.lookup_transform('world', ('/estimated_pose').lstrip('/'), node.announcement.stamp)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        
        try:
            node.chaser_pose = node.buffer.lookup_transform('world', (node.get_namespace() + '/body').lstrip('/'), node.announcement.stamp)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
            
        node.callback()
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()