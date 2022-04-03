import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSPresetProfiles

from .rigid_body_filters import filters

class RigidBodyKalman(Node):
    def __init__(self):
        super().__init__('aruco_board_estimator')
        self.pose_br = TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('verbose', 1)
        self.declare_parameter('filter_type', 'const_accel')
        self.declare_parameter('duration', False)
        
        # Get parameters
        self.filter = filters[self.get_parameter('filter_type').get_parameter_value().string_value]
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration').get_parameter_value().bool_value
        
        

        
        
def main(args=None):
    rclpy.init(args=args)
    node = RigidBodyKalman()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()