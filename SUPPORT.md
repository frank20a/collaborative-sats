# Problem Solving
Here I document most of the problems I faced and how I solved them.

## Spawn model with ROS2 Gazebo

See relevant documentation [here](https://answers.ros.org/question/314607/spawn-model-with-ros2-gazebo/).

## Publishing complicated messages from Python

Publishing messages in a topic from a Python node is not as simple as using the commandline and dictionary-like strings [e.g.](src/pose_estimation/pose_estimation/SUPPORT.md#Call-the-service-using-the-appropriate-message). We need to import the appropriate messages and service types, then set the message to a new msg object and finally change the messages primitive values. 
- [Example 1](https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist) (Outsourced)
- [Example 2](src/pose_estimation/pose_estimation/SUPPORT.md#from-a-python-node) (Self-written)

## Changing the pose of an enitity in Gazebo (by calling a ROS2 service)

See relevant document [here](src/pose_estimation/pose_estimation/SUPPORT.md).

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

## Time in ROS2 nodes

You can get the Node clock with `clock = my_node.get_clock()` and from there get the current time `time = clock.now()` and transform it to a message (e.g. for use in a header) with `time_msg = time.to_msg()`.

# Mescellaneous Notes

Notes for things that I slowly figured out how to do.

## Usage of ROS2 commands

See relevant documentation [here](/src/stereo_cam/SUPPORT.md).

## Using custom OGRE materials in models with Gazebo

See relevant documentation [here](/models/SUPPORT.md#using-ogre-materials).

## Robot Description

- URDF
  - [Links](http://wiki.ros.org/urdf/XML/link)
  - [Joints](http://wiki.ros.org/urdf/XML/joint)
- [xacro](https://www.youtube.com/watch?v=CwdbsvcpOHM&t=1090s)
- [tf](https://www.youtube.com/watch?v=QyvHhY4Y_Y8&t=2s) System
- [Gazebo & state publishers](https://www.youtube.com/watch?v=laWn7_cj434) | [More Gazebo](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/) | [Even More](https://medium.com/creating-a-gazebo-simulation-with-ros2-for-your/introduction-8daf6efa12f4)
- [SDF](http://sdformat.org/spec)

## Using launch files in ROS2

See relevant documentation [here](/src/stereo_cam/launch/SUPPORT.md).

## Creating node for ROS2 with Python3

See relevant documentation [here](/src/stereo_cam/stereo_cam/SUPPORT.md).

## Fix missaligned tf system in camera

Camera reference system in Gazebo is different than the one proposed by OpenCV and so pointclouds, projections and pose estimations performed by cameras in a simulation are missaligned and rotated. To fix that we introduce a link in the same position as the camera link but rotated as described [here](https://answers.ros.org/question/232534/gazebo-camera-frame-is-inconsistent-with-rviz-opencv-convention/). The new camera_link_optical is created as such:
```xml
<link name='camera_link_optical'></link>
<joint name="camera_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  <parent link="camera_link"/>
  <child link="camera_link_optical"/>
</joint>
```
