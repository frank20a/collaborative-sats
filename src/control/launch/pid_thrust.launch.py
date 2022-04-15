from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'pid_controller',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thrust',
        )
    )
    
    return ld

