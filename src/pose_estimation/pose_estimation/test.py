import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist

from random import random


class Tester(Node):
    def __init__(self):
        super().__init__("test_node")
        
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        else:
            print('Service found')

        self.req = SetEntityState.Request()


        self.timer = self.create_timer(1, self.send_request)
    
    def send_request(self):

        self.req.state = EntityState()

        self.req.state.name = 'marker_cube'

        self.req.state.pose = Pose()
        self.req.state.pose.position.x = random() * 4 - 2
        self.req.state.pose.position.y = random() * 4 - 2
        self.req.state.pose.position.z = random() * 2

        self.req.state.twist = Twist()

        self.req.state.reference_frame = 'world'

        self.future = self.cli.call_async(self.req)

        
        
def main(args=None):
    rclpy.init(args=args)

    tester = Tester()
    tester.send_request()

    rclpy.spin(tester)
    # while rclpy.ok():
    #     rclpy.spin_once(tester)
    #     if tester.future.done():
    #         try:
    #             response = tester.future.result()
    #         except Exception as e:
    #             tester.get_logger().info(
    #                 'Service call failed %r' % (e,))
    #         else:
    #             tester.get_logger().info('Success' if response.success else 'Failed')
    #         break

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
