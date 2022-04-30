from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    ld = LaunchDescription()
    
    # Run Gazebo simulation from control/sim.launch.py
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('pose_estimation'), 'launch'), '/sim.launch.py']),
        )
    )

    # Run PID Controller
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('control'), 'launch'), '/pid_thrust.launch.py']),
        )
    )
    
    ld.add_entity(
        Node(
            package = 'scenarios',
            executable =  'lock_target',
            # parameters=[{
            #     'verbose': 2,
            # }],
            namespace = 'chaser_0',
        )
    )
    
    return ld

