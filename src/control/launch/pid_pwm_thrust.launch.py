from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'pid_controller',
            parameters=[{
                'thruster_type': 'pwm',
                'verbose': 2,
            }],
            namespace = 'chaser_0',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thruster_pwm',
            parameters=[{
                'verbose': 2,
            }],
            namespace = 'chaser_0',
        )
    )
    
    return ld

