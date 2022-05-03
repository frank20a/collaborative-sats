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
                'verbose': 0,
                'use_sim_time': True,
                'init_setpoint': '-1.0 0.5 0.75 0.0 0.0 -0.258819 0.9659258',
            }],
            namespace = 'chaser_0',
        )
    )
    
    ld.add_entity(
        Node(
            package = 'control',
            executable =  'thruster',
            parameters=[{
                'verbose': 0,
                'use_sim_time': True,
            }],
            namespace = 'chaser_0',
        )
    )
    
    return ld

