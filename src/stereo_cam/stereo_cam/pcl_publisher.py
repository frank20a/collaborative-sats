from sympy import diag
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, CameraInfo, PointField

import cv2
from time import time
import numpy as np
import open3d as o3d


bridge = CvBridge()

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("stereo_image_viewer")

        self.pcl = o3d.geometry.PointCloud()
        self.cameraIntrinsic = o3d.camera.PinholeCameraIntrinsic()
        
        self.create_subscription(
            DisparityImage,
            '/stereo/disparity',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.publisher_pcl = self.create_publisher(
            PointCloud2, 
            'stereo/pointcloud', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        self.create_subscription(
            CameraInfo,
            '/stereo/left/camera_info',
            self.set_camera_intrinsic,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

    def callback(self, msg: DisparityImage):
        if self.cameraIntrinsic.is_valid():
            img = bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            
            depth = msg.t * msg.f / img
            
            self.pcl.clear()
            self.pcl.create_from_depth_image(depth, self.cameraIntrinsic)
            points = np.asarray(self.pcl.points)

            self.publisher_pcl.publish(
                PointCloud2(
                    header = Header(frame_id="camera_midpoint"),
                    height = points.shape[1], 
                    width = points.shape[0],
                    is_dense = False,
                    is_bigendian = False,
                    fields = [
                        PointField(
                            name = n, 
                            offset = i * np.dtype(np.float32).itemsize, 
                            datatype = PointField.FLOAT32, 
                            count = 1
                        ) for i, n in enumerate('xyz')
                    ],
                    point_step = (np.dtype(np.float32).itemsize, * 3),
                    row_step = (np.dtype(np.float32).itemsize, * 3 * points.shape[0]),
                    data = points.astype(np.float32).tobytes()
                )
            )

    def set_camera_intrinsic(self, msg: CameraInfo):
        self.cameraIntrinsic.set_intrinsics(
            msg.width,
            msg.height,
            msg.k[0],
            msg.k[4], 
            msg.k[2], 
            msg.k[5]
        )


def main(args=None):
    rclpy.init(args=args)
    viewer = PointCloudPublisher()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
