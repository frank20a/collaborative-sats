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
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('scenarios'), 'launch'), '/sim.launch.py']),
        )
    )

    # Run Controller
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('control'), 'launch'), '/mpc_thrust.launch.py']),
        )
    )
    
    ld.add_entity(
        Node(
            package = 'scenarios',
            executable =  'pose_match',
            namespace = 'chaser_0',
        )
    )
    
    return ld

