import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import QoSPresetProfiles

import numpy as np


abs = lambda x: x if x > 0 else -x
floor_ = lambda x, n=1e-4: x if abs(x) > n else 0.0
sgn = lambda x: (1 if x > 0 else -1) if floor_(x, 1e-2) != 0 else 0


class CombineEstimations(Node):
    def __init__(self):
        super().__init__('combine_estimations')
        self.broadcaster = TransformBroadcaster(self)
        
        self.declare_parameter('num_chasers', 2)
        self.declare_parameter('freq', 30.0)
        self.declare_parameter('verbose', 0)
        self.declare_parameter('topics', False)
        self.declare_parameter('ra_len', 11)
        self.declare_parameter('prefix', 'chaser')
        
        self.num_chasers = self.get_parameter('num_chasers').get_parameter_value().integer_value
        self.dt = 1 / self.get_parameter('freq').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        self.topics = self.get_parameter('topics').get_parameter_value().bool_value        
        self.ra_len = self.get_parameter('ra_len').get_parameter_value().integer_value
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value

        
        # create a tf2 buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        if self.topics:
            # self.p_pub = self.create_publisher(Pose, 'estimated_pose', QoSPresetProfiles.get_from_short_key('sensor_data'))
            # self.t_pub = self.create_publisher(Twist, 'estimated_twist', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.pub = self.create_publisher(Odometry, 'estimated_odom', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.create_subscription(Empty, 'toggle_avg_twist', self.avg_callback, QoSPresetProfiles.get_from_short_key('sensor_data'))

            self.prev_vels = None
            self.prev_state = None
            self.avg = False
        
        self.tfs = [None for i in range(self.num_chasers)]

        self.create_timer(self.dt, self.combine_n_publish)

    def avg_callback(self, msg):
        self.avg = not self.avg
        if self.verbose > 0: self.get_logger().info('Toggled avg to {}'.format(self.avg))

    def combine_n_publish(self):
        # create a combined transform
        combined_transform = TransformStamped()
        combined_transform.header.stamp = self.get_clock().now().to_msg()
        combined_transform.header.frame_id = 'world'
        combined_transform.child_frame_id = 'estimated_pose'
        
        euler_avg_num = np.array([0, 0, 0], dtype=np.float32)
        euler_avg_denum = np.array([0, 0, 0], dtype=np.float32)
        trans_avg = np.array([0, 0, 0], dtype=np.float32)
        
        c = 0
        for i in range(self.num_chasers):
            if self.tfs[i] is None: continue

            c += 1
            transform = self.tfs[i]

            trans_avg += np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ], dtype=np.float32)
            
            quat = [
                transform.transform.rotation.x, 
                transform.transform.rotation.y, 
                transform.transform.rotation.z, 
                transform.transform.rotation.w
            ]
            euler_avg_num += np.sin(np.array(euler_from_quaternion(quat)))
            euler_avg_denum += np.cos(np.array(euler_from_quaternion(quat)))

            self.tfs[i] = None

        if c == 0: return
            
        trans_avg /= c
        euler_avg = np.arctan2(euler_avg_num, euler_avg_denum)
        quat_avg = quaternion_from_euler(euler_avg[0], euler_avg[1], euler_avg[2])
        
        combined_transform.transform.translation.x = float(trans_avg[0])
        combined_transform.transform.translation.y = float(trans_avg[1])
        combined_transform.transform.translation.z = float(trans_avg[2])
        
        combined_transform.transform.rotation.x = float(quat_avg[0])
        combined_transform.transform.rotation.y = float(quat_avg[1])
        combined_transform.transform.rotation.z = float(quat_avg[2])
        combined_transform.transform.rotation.w = float(quat_avg[3])
        
            
        # publish the combined transform
        if self.verbose > 0: self.get_logger().info('Publishing combined transform')
        self.broadcaster.sendTransform(combined_transform)

        if self.topics:
            self.publish_state(combined_transform)

    def publish_state(self, combined_transform):
        msg = Odometry()

        p = Pose()
        p.position.x = combined_transform.transform.translation.x
        p.position.y = combined_transform.transform.translation.y
        p.position.z = combined_transform.transform.translation.z
        p.orientation.x = combined_transform.transform.rotation.x
        p.orientation.y = combined_transform.transform.rotation.y
        p.orientation.z = combined_transform.transform.rotation.z
        p.orientation.w = combined_transform.transform.rotation.w
        # self.p_pub.publish(p)
        msg.pose.pose = p

        t = Twist()
        eul = np.array(euler_from_quaternion([
            combined_transform.transform.rotation.x,
            combined_transform.transform.rotation.y, 
            combined_transform.transform.rotation.z, 
            combined_transform.transform.rotation.w])
        )
        if self.prev_state is None or self.prev_vels is None: 
            v = np.zeros(3)
            omega = np.zeros(3)
            self.prev_vels = np.zeros((1, 6))
        else:
            # Calculate velocity
            v = [
                (combined_transform.transform.translation.x - self.prev_state[0]) / self.dt,
                (combined_transform.transform.translation.y - self.prev_state[1]) / self.dt,
                (combined_transform.transform.translation.z - self.prev_state[2]) / self.dt,
            ]                   

            # Calculate omega
            try:
                eul_diff = eul - self.prev_state[3:6]
            except IndexError:
                self.get_logger().info("{}".format(self.prev_state))
                raise Exception()
            omega = eul_diff / self.dt

            # Post-process velocity
            for i in range(3):
                v[i] = v[i] if abs(v[i] - self.prev_vels[-1][i]) < 3 else self.prev_vels[-1][i]
                # v[i] = floor_(v[i], 0.1)
                omega[i] = omega[i] if abs(omega[i] - self.prev_vels[-1][i + 3]) < 1.5 else self.prev_vels[-1][i + 3]
                # omega[i] = floor_(omega[i], 0.035)
        
        self.prev_state = np.array([
            p.position.x,
            p.position.y,
            p.position.z,
            eul[0],
            eul[1],
            eul[2]
        ])
        self.prev_vels = np.concatenate((self.prev_vels, np.array([[
            v[0],
            v[1],
            v[2],
            omega[0],
            omega[1],
            omega[2]
        ]], dtype=np.float32)))
        self.prev_vels = self.prev_vels[-150:]

        ra_len = min(len(self.prev_vels), self.ra_len)
        t.linear.x = floor_(np.average(self.prev_vels[-ra_len:, 0]))
        t.linear.y = floor_(np.average(self.prev_vels[-ra_len:, 1]))
        t.linear.z = floor_(np.average(self.prev_vels[-ra_len:, 2]))
        t.angular.x = floor_(np.average(self.prev_vels[-ra_len:, 3]))
        t.angular.y = floor_(np.average(self.prev_vels[-ra_len:, 4]))
        t.angular.z = floor_(np.average(self.prev_vels[-ra_len:, 5]))

        # self.t_pub.publish(t)
        msg.twist.twist = t

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CombineEstimations()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        for i in range(node.num_chasers):
            if node.tfs[i] is not None: continue
            
            try:
                node.tfs[i] = node.buffer.lookup_transform('world', node.prefix + f'_{i}/estimated_pose', Time(seconds=0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
            
        # if all(map(lambda x: x is not None, node.tfs)):
        #     node.combine_n_publish()
        #     node.tfs = [None for i in range(node.num_chasers)]
    
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()