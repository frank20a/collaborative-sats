import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Int8MultiArray

class ThrustTest(Node):
    def __init__(self):
        super().__init__('thrust_test')

        self.pub = self.create_publisher(Int8MultiArray, '/thrusters', QoSPresetProfiles.get_from_short_key('sensor_data'))

        self.create_timer(1, self.timer_callback)
        self.thruster = 0
        self.flag = True
        
    def timer_callback(self):
        res = Int8MultiArray()
        if not self.flag:
            res.data = [0] * 8
            self.get_logger().info(' OFF: ' + ['11', '12', '21', '22', '31', '32', '41', '42'][self.thruster])
            self.flag = True
        else:
            res.data = [1 if i == self.thruster else 0 for i in range(8)]
            self.thruster = (self.thruster + 1) % 8
            self.get_logger().info('  ON: ' + ['11', '12', '21', '22', '31', '32', '41', '42'][self.thruster])
            self.flag = False

        self.pub.publish(res)
        

def main(args=None):
    rclpy.init(args=args)
    node = ThrustTest()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()