import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSPresetProfiles
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

from .filter_type import filters


class RigidBodyKalman(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.pose_br = TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('verbose', 1)
        self.declare_parameter('filter_type', 'const_accel')
        self.declare_parameter('duration', False)
        self.declare_parameter('state_indexes', '0,1,2,9,10,11')
        
        # Get parameters
        self.filter = filters[self.get_parameter('filter_type').get_parameter_value().string_value](1/60)
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration').get_parameter_value().bool_value
        self.state_indexes = [int(i) for i in self.get_parameter('state_indexes').get_parameter_value().string_value.split(',')]
        
        
        self.create_subscription(
            TransformStamped,
            'pose_estimation',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.publisher_disp = self.create_publisher(
            TransformStamped, 
            'filtered_estimation', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

    def callback(self, t):
        euler = np.array(euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]), dtype=np.float32)
        trans = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], dtype=np.float32)
        
        self.filter.predict()
        self.filter.update(np.concatenate((trans, euler)))
        
        t.transform.translation.x = self.filter.x[self.state_indexes[0]]
        t.transform.translation.y = self.filter.x[self.state_indexes[1]]
        t.transform.translation.z = self.filter.x[self.state_indexes[2]]
        
        euler = quaternion_from_euler(self.filter.x[self.state_indexes[3]], self.filter.x[self.state_indexes[4]], self.filter.x[self.state_indexes[5]])
        t.transform.rotation.x = euler[0]
        t.transform.rotation.y = euler[1]
        t.transform.rotation.z = euler[2]
        
        t.child_frame_id = (self.get_namespace() + '/filtered_estimation').lstrip('/')
        self.pose_br.sendTransform(t)
        self.publisher_disp.publish(t)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = RigidBodyKalman()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()