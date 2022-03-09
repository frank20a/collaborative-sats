# collaborative-sats
Part of my thesis. My ROS2 workspace for simulations.

## Packages

### Stereo Vision

The `stereo_cam` package contains scripts and simulation environments for extracting 3D pointclouds from a stereoscopic camera. Simulations are set up on Gazebo and the results presented on RViz2.

### Pose Estimation

The `pose_estimation` package contains usefull methods for estimating the pose of the target. Starting from the easiest method with prior knowledge about the target considered, I move towards a method involving more and more autonomy. 

#### ArUco markers

Software tracks ArUco markers on target using a calibrated camera and extracts pose. 

## Build
```console
colcon build --symlink-install
```

## Use
```console
source ./install/setup.bash
ros2 launch <package_name> <launch_file.py>
ros2 run <package_name> <node.py>
```
