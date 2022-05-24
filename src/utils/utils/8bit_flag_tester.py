import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.qos import QoSPresetProfiles

class SetpointPub(Node):
    def __init__(self):
        super().__init__('flag_tester')

        self.create_subscription(Int16, 'thruster_flags', self.callback, QoSPresetProfiles.get_from_short_key('sensor_data'))
        
    def callback(self, msg: Int16):
        self.get_logger().info(bin(msg.data)[2:].rjust(8, '0')[::-1] + f"   -   {msg.data}")
        
        

def main(args=None):
    rclpy.init(args=args)
    node = SetpointPub()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()