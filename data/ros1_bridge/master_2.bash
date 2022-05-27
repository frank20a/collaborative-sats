export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_IP=192.168.1.15
source /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
# source /your/ros2/ws/directory/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics