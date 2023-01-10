import rclpy, numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSPresetProfiles
from functools import partial
from scipy.spatial.transform import Rotation as R

offsets = {
    'chaser_0': [-1, 0, 0, 0, 0, 0, 1],
    'chaser_1': [0, -1, 0, 0, 0, 0.7071, 0.7071],
    'chaser_2': [0,  1, 0, 0, 0, 0.7071, -0.7071],
    'chaser_3': [1, 0, 0, 0, 0, 1, 0]
}

def qmul(q0, q1):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
     
    return np.array([Q0Q1_x, Q0Q1_y, Q0Q1_z, Q0Q1_w])

class Pather(Node):
    def __init__(self):
        super().__init__('pather')

        self.paths = {
            'chaser_0': Path(),
            'chaser_1': Path(),
            'chaser_2': Path(),
            'chaser_3': Path(),
            'target': Path(),
            # 'estim': Path(),
        }
        self.pubs = {}
        self.ref_pubs = {}
        self.ref_paths = {
            'chaser_0': Path(),
            'chaser_1': Path(),
            'chaser_2': Path(),
            'chaser_3': Path(),
            'target': Path(),
        }
        for name in self.paths:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                partial(self.callback, name),
                QoSPresetProfiles.get_from_short_key('sensor_data')
            )
            self.paths[name].header.frame_id = 'world'
            self.ref_paths[name].header.frame_id = 'world'
            self.pubs[name] = self.create_publisher(Path, f'/{name}/path', QoSPresetProfiles.get_from_short_key('sensor_data'))
            self.ref_pubs[name] = self.create_publisher(Path, f'/{name}/ref_path', QoSPresetProfiles.get_from_short_key('sensor_data'))

        self.create_subscription(String, '/pather_cmd', self.cmd_callback, QoSPresetProfiles.get_from_short_key('system_default'))

    def make_n_pub_ref(self, name):
        tar = self.paths['target'].poses[-1]
        
        p = np.array([tar.pose.position.x, tar.pose.position.y, tar.pose.position.z])
        q = np.array([tar.pose.orientation.x, tar.pose.orientation.y, tar.pose.orientation.z, tar.pose.orientation.w])

        p = p + R.from_quat(q).apply(offsets[name][:3])
        q = qmul(q, offsets[name][3:])
        
        tmp = PoseStamped()
        tmp.pose.position.x = p[0]
        tmp.pose.position.y = p[1]
        tmp.pose.position.z = p[2]
        tmp.pose.orientation.x = q[0]
        tmp.pose.orientation.y = q[1]
        tmp.pose.orientation.z = q[2]
        tmp.pose.orientation.w = q[3]
        tmp.header = tar.header
        self.ref_paths[name].poses.append(tmp)
        self.ref_paths[name].header.stamp = tar.header.stamp
        self.ref_pubs[name].publish(self.ref_paths[name])


    def callback(self, name: str, msg: Odometry):
        tmp = PoseStamped()
        tmp.pose = msg.pose.pose
        tmp.header = msg.header
        self.paths[name].poses.append(tmp)
        self.paths[name].header.stamp = msg.header.stamp
        self.pubs[name].publish(self.paths[name])

        if name == 'target':
            self.make_n_pub_ref('chaser_0')
            self.make_n_pub_ref('chaser_1')
            self.make_n_pub_ref('chaser_2')
            self.make_n_pub_ref('chaser_3')

    def cmd_callback(self, msg: String):
        if msg.data == 'reset':
            self.get_logger().info("Resetting paths")
            for name in self.paths:
                self.paths[name].poses = []
                self.ref_paths[name].poses = []
                self.pubs[name].publish(self.paths[name])
                self.ref_pubs[name].publish(self.ref_paths[name])


def main(args=None):
    rclpy.init(args=args)
    node = Pather()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()