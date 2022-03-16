# Some notes for controlling the pose of entities in Gazebo from ROS2
You need to add a plugin to the world SDF file inside the `<world>` tag
```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <!-- <ros>
        <namespace>/chaser</namespace>
        <argument>model_states:=model_states_demo</argument>
    </ros> -->
    <update_rate>1.0</update_rate>
</plugin>
```

Then call the `/set_entity_state` service with the new pose message
```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState '{state:{name: marker_cube, pose: { position: { x: 1, y: 0 ,z: 1 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 }}, reference_frame: world }}'
```

