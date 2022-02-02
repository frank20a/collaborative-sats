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

        self.cameraIntrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 720,
            height = 480,
            fx = 623.6805519254747,
            fy = 623.6805519254747,
            cx = 360.5,
            cy = 240.5,
        )
        
        self.create_subscription(
            DisparityImage,
            '/stereo/disparity',
            self.callback,
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )
        
        self.publisher_pcd = self.create_publisher(
            PointCloud2, 
            'stereo/pointcloud', 
            QoSPresetProfiles.get_from_short_key('sensor_data')
        )

        # self.create_subscription(
        #     CameraInfo,
        #     '/stereo/left/camera_info',
        #     self.set_camera_intrinsic,
        #     QoSPresetProfiles.get_from_short_key('sensor_data')
        # )

        pass

    def callback(self, msg: DisparityImage):
        if self.cameraIntrinsic.is_valid():
            disparity = bridge.imgmsg_to_cv2(msg.image)
            # disparity = disparity - 16.0 * (msg.min_disparity - 1)
            
            depth_map = 10 * msg.t * msg.f / disparity
            # print(np.min(depth_map), np.max(depth_map))
            depth = o3d.geometry.Image(depth_map)
            
            pcd = o3d.geometry.PointCloud().create_from_depth_image(
                depth, 
                self.cameraIntrinsic,
                extrinsic = np.array([
                    [ 0,  0, -1,  0],
                    [ 0,  1,  0,  0],
                    [ 1,  0,  0,  0],
                    [ 0,  0,  0,  1]
                ])
            )
            pcd.transform([
                [ 1,  0,  0,  0],
                [ 0,  0,  1,  0],
                [ 0, -1,  0,  0],
                [ 0,  0,  0,  1]
            ])
            # o3d.visualization.draw_geometries([pcd])

            points = np.asarray(pcd.points)

            self.publisher_pcd.publish(
                PointCloud2(
                    header = Header(
                        frame_id="camera_midpoint"
                    ),
                    height = 1, 
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
                    point_step = (np.dtype(np.float32).itemsize * 3),
                    row_step = (np.dtype(np.float32).itemsize * 3 * points.shape[0]),
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
        # print(self.cameraIntrinsic.intrinsic_matrix)


def main(args=None):
    rclpy.init(args=args)
    viewer = PointCloudPublisher()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
