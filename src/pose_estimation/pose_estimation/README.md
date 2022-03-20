# Some notes for controlling the pose of entities in Gazebo from ROS2

## Use the gazebo_ros plugin

You need to add a plugin to the world SDF file inside the `<world>` tag
```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <!-- <ros>
        <namespace>/chaser</namespace>
        <argument>model_states:=model_states_demo</argument>
    </ros> -->
    <update_rate>1.0</update_rate>
</plugin>
```

## Call the service using the appropriate message

Then call the `/set_entity_state` service with the new pose message
```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState '{state:{name: marker_cube, pose: { position: { x: 1, y: 0 ,z: 1 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 }}, reference_frame: world }}'
```

## From a Python node

This can be done from a Python node. We need to keep in mind the following messages structure
- [SetEntityState service](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_msgs/srv/SetEntityState.srv)
- [EntityState message](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_msgs/msg/EntityState.msg)

And [how to](https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist) send such messages

```py
import rclpy
from rclpy.node import Node

# We need to improt the messages we will use
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist


class Tester(Node):
    def __init__(self):
        super().__init__("test_node")

        # Creating a client for the Gazebo service
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        # Waiting for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service to be available...')
        else:
            print('Service found')

        # Create a request to the service
        self.req = SetEntityState.Request()
    
    def send_request(self):

        # Fill the request according to the documentation for the messages
        self.req.state = EntityState()

        self.req.state.name = 'marker_cube'

        self.req.state.pose = Pose()
        self.req.state.pose.position.x = 2.0
        self.req.state.pose.position.y = 2.0
        self.req.state.pose.position.z = 2.0

        self.req.state.twist = Twist()

        self.req.state.reference_frame = 'world'

        # Create the future (to tell rclpy to stop spinning) by sending the request
        self.future = self.cli.call_async(self.req)

        
        
def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    tester = Tester()
    tester.send_request()

    while rclpy.ok():
        # Spin the node and wait for its future
        rclpy.spin_once(tester)
        if tester.future.done():
            try:
                # Get the response
                response = tester.future.result()
            except Exception as e:
                tester.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # print the respnose to the logger
                tester.get_logger().info('Success' if response.success else 'Failed')
            break

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
