from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    ld = LaunchDescription()
    
    
    # ================= Simulation =================
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('scenarios'), 'launch'), '/sim.launch.py']),
        )
    )


    # ================= Estimation Combiner =================
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable =  'combine_estimations',
            parameters=[{
                'use_sim_time': True,
                'num_chasers': 1,
            }]
        )
    )


    # ================= Controller =================
    ld.add_entity(
        Node(
            package = 'control',
            executable = 'pose_match',
        )
    )

    for i in range(1):
        ld.add_entity(
            Node(
                package = 'control',
                executable = 'thruster_pwm',
                namespace = 'chaser_' + str(i),
            )
        )


    # ================= RViz =================
    ld.add_entity(
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            output = {'both': 'log'},
            parameters = [{
                'use_sim_time': True
            }],
            arguments = [
                '-d', os.path.join(get_package_share_directory('scenarios'), 'rviz_config/scenarios2.rviz'),
            ],
        )
    )
    
    
    return ld

