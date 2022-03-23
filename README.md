# collaborative-sats
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

## Problem Solving
Here I document most of the problems I faced and how I solved them.

### Spawn model with ROS2 Gazebo

See relevant documentation [here](https://answers.ros.org/question/314607/spawn-model-with-ros2-gazebo/)

### Publishing complicated messages from Python

Publishing messages in a topic from a Python node is not as simple as using the commandline and dictionary-like strings [e.g.](src/pose_estimation/pose_estimation/README.md#Call-the-service-using-the-appropriate-message). We need to import the appropriate messages and service types, then set the message to a new msg object and finally change the messages primitive values. 
- [Example 1](https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist) (Outsourced)
- [Example 2](src/pose_estimation/pose_estimation/README.md#from-a-python-node) (Self-written)

### Robot Description

- URDF
  - [Links](http://wiki.ros.org/urdf/XML/link)
  - [Joints](http://wiki.ros.org/urdf/XML/joint)
- [xacro](https://www.youtube.com/watch?v=CwdbsvcpOHM&t=1090s)
- [tf](https://www.youtube.com/watch?v=QyvHhY4Y_Y8&t=2s) System
- [Gazebo & state publishers](https://www.youtube.com/watch?v=laWn7_cj434) | [More Gazebo](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/) | [Even More](https://medium.com/creating-a-gazebo-simulation-with-ros2-for-your/introduction-8daf6efa12f4)

### Changing the pose of an enitity in Gazebo (by calling a ROS2 service)

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
- [Send geometry (and other types) of messages from Python](https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist)

### Time in ROS2 nodes

You can get the Node clock with `clock = my_node.get_clock()` and from there get the current time `time = clock.now()` and transform it to a message (e.g. for use in a header) with `time_msg = time.to_msg()`.

### Using custom materials in models with Gazebo

See relevant documentation [here](/models/README.md)
