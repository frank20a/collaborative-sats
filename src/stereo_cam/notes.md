# General notes on ROS2 packages & commands

## Create workspace
`colcon build --symlink-install`
## Create package inside workspace
`ros2 pkg create --build-type <build_type> <package_name>`

        --build-type ament_python/ament_cmake       
        --node-name <node_name>                     Create Hello World executable

## Build package
`colcon build --symlink-install`
        
        Build the packages
        --packages-select <packages>
        Doesn't need to build every time I change files
        --symlink-install

## Launch gazebo with ros integration
`ros2 launch gazebo_ros gazebo.launch.py`

## Run robot state publisher
`ros2 run robot_state_publisher --ros-args -p robot_description:="$(xacro ~/test.xacro)"`

## Run joint_state_publisher
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Run RVIZ (configuration file)
`rviz2 (-d src/view.rviz)`
