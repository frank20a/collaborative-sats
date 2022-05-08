from launch import LaunchDescription
from launch_ros.actions import Node

from control.pose_match_mpc_generator.parameters import nc

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_entity(
        Node(
            package = 'control',
            executable =  'pose_match',
            parameters=[{
                'nc': nc,
            }],
            namespace = 'chaser_0',
        )
    )
    
    for i in range(nc):
        ld.add_entity(
            Node(
                package = 'control',
                executable =  'thruster_pwm',
                parameters=[{
                    'verbose': 0,
                    # 'use_sim_time': True,
                }],
                namespace = 'chaser_' + str(i),
            )
        )
    
    return ld

