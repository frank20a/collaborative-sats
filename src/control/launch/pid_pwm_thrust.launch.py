from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'pid',
            parameters=[{
                'thruster_type': 'pwm',
                'verbose': 2,
                'use_sim_time': True,
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
                'use_sim_time': True,
            }],
            namespace = 'chaser_0',
        )
    )
    
    return ld

