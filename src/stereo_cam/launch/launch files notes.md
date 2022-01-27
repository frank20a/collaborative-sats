# Notes on launch files

## Bare minimum
```py
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            # === Executable name ===
            name = 'state_publisher',
            # === Function parameters (Would be robot_description:="...") ===
            parameters = [{'robot_description': raw_robot}],
            # === Function call parameters ===
            arguments=['-topic', 'stereo_cam_desc', '-entity', 'stereo']
            # === Remapping topic names ===
            remappings=[('default', 'new'), ]
            # === Verbose output (file or screen etc.) ===
            output = 'screen',
        ),
    ])
```
## Launch from launch file
```py
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
),
```
## Getting arguments
```py
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

robot_path = LaunchConfiguration('robot')
robot_path_launch_arg = DeclareLaunchArgument(
    'robot',
    default_value='idfk'
)
```
---> use robot_path as substitution