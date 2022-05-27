# Bridging ROS1 and ROS2 remotely

Instructions to connect a computer running ROS2 to a robot running ROS1. For example Laptop (ROS2 Galactic) to Slider (ROS1 Melodic).

ROS2 (master) --- ros1_bridge ---> ROS1 (master) --- network ---> ROS1 (slave)

## On the ROS2 master computer (Laptop w/ ROS2)
1. Install ROS1 on the master computer. Only basic ROS1 packages are required. For example, the following command installs ROS Noetic package.
```bash
sudo apt install ros-noetic-ros-base
```
2. Install ros1_bridge. For example, the following command installs ros1_bridge on ROS2 Galactic.
```bash
sudo apt install ros-galactic-ros1-bridge
```
3. Restart all terminal and source your workspaces again.
4. Run `hostname -I` to find the master computers IP address.
5. Run the following script to setup the computer as a ROS1 master (Replace .15 IP addess with the master computers IP address and Noetic with the ROS1 version you have installed).
```bash
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_IP=192.168.1.15
source /opt/ros/noetic/setup.bash
roscore
```
6. In another terminal run this command to start ros1_bridge (Replace noetic and galactic with the install ROS1 and ROS2 versions respectively AND the IP with the master computers IP address).
```bash
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_IP=192.168.1.15
source /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
source /your/ros2/ws/directory/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

## On the ROS1 slave computer (Single-chip computer on the Slider w/ ROS1)

0. Run `hostname -I` to get the slave computers IP address
1. Run the following script setup the connection and test it (Change the .18 IP with the slaves IP and the .15 with the masters)
```bash
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_IP=192.168.1.18
rostopic list
```

## Run a test

1. On the master ROS2 computer open a terminal and run:
```bash
ros2 topic pub /test std_msgs/msg/String 'data: "Hello World!"'
```
You shoud see a message on the ros1_bridge terminal saying that it detected a new topic and is translating it to ROS1

2. On the slave ROS1 computer open a terminal and run:
```bash
rostopic echo /test
```
You should be seeing "Hello World!" messages being received.
