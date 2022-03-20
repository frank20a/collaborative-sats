# collaborative-sats
Part of my thesis. My ROS2 workspace for simulations. I'm using the latest Galactic version of ROS2.

## Packages

### Stereo Vision

The `stereo_cam` package contains scripts and simulation environments for extracting 3D pointclouds from a stereoscopic camera. Simulations are set up on Gazebo and the results presented on RViz2.

### Pose Estimation

The `pose_estimation` package contains usefull methods for estimating the pose of the target. Starting from the easiest method with prior knowledge about the target considered, I move towards a method involving more and more autonomy.

#### ArUco markers

Software tracks ArUco markers on target using a calibrated camera and extracts pose. To test the usage you can use the `aruco.launch.py` launch file and test the pose estimation from the computers built-in camera.
```console
ros2 launch pose_estimation aruco.launch.py
```

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

## Problem Solving
Here I document most of the problems I faced and how I solved them.

### Changing the pose of an enitity in Gazebo
See relevant document [here](src/pose_estimation/pose_estimation/README.md)
Sources (Don't count on them indicidually... they are for different ROS versions):
- [ROS Communication](http://gazebosim.org/tutorials/?tut=ros_comm)
- [ROS Integration overview](http://gazebosim.org/tutorials?tut=ros_overview)
- [ROS2 Integration overview](http://gazebosim.org/tutorials?tut=ros2_overview)
- [Using gazebo_ros plugins in ROS2](https://answers.ros.org/question/356936/how-to-use-gazebo-plugins-found-in-gazebo_ros-ros2-foxy-gazebo11/)
- [Using gazebo_ros API (get the `get_entity_state` service to show up) in ROS2](https://answers.ros.org/question/360161/ros2-dashing-service-get_entity_state-is-missing/)
- [Creating a service client in ROS2](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html)
- [List of gazebo_msgs](https://index.ros.org/p/gazebo_msgs/#galactic-assets)
  - [SetEntityState service](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_msgs/srv/SetEntityState.srv)
  - [EntityState message](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/galactic/gazebo_msgs/msg/EntityState.msg)