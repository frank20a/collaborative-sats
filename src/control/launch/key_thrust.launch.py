from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'key_teleop',
            namespace = 'chaser_0',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thruster',
            namespace = 'chaser_0',
        )
    )
    
    return ld

