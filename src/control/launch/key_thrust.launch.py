from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'key_teleop',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thrust',
        )
    )
    
    return ld

