# rclpy notes

[rclpy API documentation](https://docs.ros2.org/latest/api/rclpy/index.html)

## Coding the node

### Minimal publisher

```py
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

Imports the Node class and String message type

```py
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

- Class is subclassing Node
- Node contains a publisher
- Node contains a timer that calls the timer_callback() function every period
- timer_callback() creates a String message and uses the Node publisher to send it

```py
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- main() function is called upon running the executable.
- main() initializes rclpy and spins the publisher Node
- the Node gets destroyed in the end and rclpy shuts down

### Minimal Subscriber

The subscriber class is as follows

```py
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

```
- Node contains a subscription to a topic with a callback on message received and a given [qos_profile](https://docs.ros2.org/latest/api/rclpy/api/qos.html) (10 = RMW_QOS_POLICY_HISTORY_KEEP_LAST with history depth of 10)

## Adding dependencies

Need to add following lines in `package.xml` to reflect the corresponding node's import statements.

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

## Adding entry point

Need to change the 'setup.py' file to include the entry point for each python script.

```py
entry_points={
        'console_scripts': [
                '<entry_point> = ' + package_name + '.<python_filename>:main',
        ],
},
```