from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'pid',
            parameters=[{
                'thruster_type': 'onoff',
                'verbose': 1,
            }],
            namespace = 'chaser_0',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thruster',
            parameters=[{
                'verbose': 1,
            }],
            namespace = 'chaser_0',
        )
    )
    
    return ld

