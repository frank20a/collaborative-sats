# Future Development

- Kalman Filtering
  - Filtering pose estimation output from each chaser
  - Sensor fusion from the different chasers
  - Improving on the estimation model to include the chasers plant
- Controllers
  - Add controllers to the chaser so I can move them arround by hand
  - Controller for visual locking
  - Controller for attitude synchronization
  - Controller for docking
- Visual Pose Estimation Methods (using features instead of markers)

# Changelog

## 27 March 2022 dd7153b03bb853ee56379ad59e1d8e7d19ba5308

Managed to launch multiple chasers in the simulation with their own namespaces that seperately estimate the pose of the target.

## 22 March 2022 553a800ae016fb883fada1b197e245c0243f89e6

Pose estimation works fine with ArUco markers over the target. The position of the markers must be known apriori. This is done using the ArUco Board method implemented in OpenCV.

## 21 March 2022 2d4ca4899f414b41ddaee0bad7b551ab27054ec2

Automatic camera calibration in Gazebo using the `calibrate_sim` node and the the `sim_calibration.launch.py` launch file.

## 15 March 2022 cc4439c1af3700612178583113c9fe4754385791

Can now have a target with custom materials (markers) spawned into Gazebo.

## 9 March 2022 979886705f17e9a66797183dfa18922cf3803a58

Started working on the package `pose_estimation`

Can now detect the pose of ArUco markers from the computer camera using the `aruco` node. This is done after calibrating the camera using the `calibrate` node.

## 2 February 2022 e92d5fc9dce26a04bfe1bde739d6882c1e1a2c68

Finished the `pcd_publisher` node that translates a disparity map to a point cloud and publishes it through ROS2 in a PointCloud2 message.

## 31 January 2022 2490e54b173c4dcdda1ea3f285e687537d985afa

Created `disparity_publisher` and `disparity_viewer` nodes that translate images from a stereoscopic camera into a disparity map and then publish or view it as a greyscale image.
