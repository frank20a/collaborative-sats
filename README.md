# collaborative-sats
![GitHub](https://img.shields.io/github/license/frank20a/collaborative-sats?style=flat-square)  ![GitHub issues](https://img.shields.io/github/issues/frank20a/collaborative-sats?style=flat-square)  ![GitHub closed issues](https://img.shields.io/github/issues-closed/frank20a/collaborative-sats?style=flat-square)  ![GitHub commit activity](https://img.shields.io/github/commit-activity/m/frank20a/collaborative-sats?style=flat-square)

Part of my thesis. My ROS2 workspace for simulations. I'm using the latest [Galactic](https://docs.ros.org/en/galactic/index.html) version of ROS2.

## Packages

Info about the nodes in each package can be found in the README files of each package folder (Click the links in the titles).

### Stereo Vision

The `stereo_cam` package contains scripts and simulation environments for extracting 3D pointclouds from a stereoscopic camera. Simulations are set up on Gazebo and the results presented on RViz2.

### (Pose Estimation)[/src/pose_estimation]

The `pose_estimation` package contains usefull methods for estimating the pose of the target. Starting from the easiest method with prior knowledge about the target considered, I move towards a method involving more and more autonomy.

#### ArUco markers

Software tracks ArUco markers on target using a calibrated camera and extracts pose. To test the usage you can use the `aruco.launch.py` launch file and test the pose estimation from the ~~computers built-in~~ from the chaser camera in a Gazebo simulation environment.
```console
ros2 launch pose_estimation aruco.launch.py
```

#### Calibration

Automatically loads lighting and a chessboard in a Gazebo world. The script automaticaly moves the chessboard arround, takes pictures using the chasers camera and creates the calibration parameters in a JSON file. The whole process is automated and **relatively** quick (see #1).

## Build

Run from the workspace directory
```console
colcon build --symlink-install
```

## Use
```console
source ./install/setup.bash
ros2 launch <package_name> <launch_file.py>
ros2 run <package_name> <node.py>
```
