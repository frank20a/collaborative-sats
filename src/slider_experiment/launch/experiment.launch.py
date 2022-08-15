from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import xacro, os

raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/slider/slider.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
nc = 1

def generate_launch_description():

    ld = LaunchDescription()    

    # ================= Vicon =================
    # Vicon Receiver
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('vicon_receiver'), 'launch'), '/client.launch.py']),
        )
    )

    # Vicon Translators
    ld.add_entity(
        Node(
            package = 'slider_experiment',
            executable = 'vicon_filter',
            name = 'vicon_slider',
            remappings=[
                ('input', 'vicon/slider_0605/slider_0605'),
                ('output', 'slider_0/odom'),
            ],
            parameters=[{
                'master': True,
            }]
        )
    )
    
    # ================= Chasers =================
    # Controller
    ld.add_entity(
        Node(
            package = 'slider_experiment',
            executable = 'slider_mpc',
            parameters=[{
                'nc': nc,
                'verbose': 2,
                'dock_vel': 0.1,
                'dock_dist': 0.05,
            }],
        )
    )

    for i in range(nc):
        ns = 'slider_' + str(i)

        # Publish chaser to robot_state_publisher
        ld.add_entity(
            Node(
                package = 'robot_state_publisher',
                executable =  'robot_state_publisher',
                name = ns + '_state_publisher',
                parameters = [{
                    'robot_description': raw_chaser,
                    'use_sim_time': True,
                    'frame_prefix': ns + '/'
                }],
                namespace = ns
            )
        )

        # Controller
        ld.add_entity(
            Node(
                package = 'slider_experiment',
                executable = 'tsl',
                namespace = ns,
                parameters=[{
                    'frequency': 1,
                    'resolution': 10
                }]
            ),
        )

        # Raw Estimation converter
        ld.add_entity(
            Node(
                package = 'slider_experiment',
                executable = 'pose_raw_converter',
                namespace = ns
            ),
        )



    # Estimation combiner
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable = 'combine_estimations',
            parameters =[{
                'ra_len': 5,
                'duration': False,
                'use_sim_time': False,
                'num_chasers': nc,
                'topics': True,
                'prefix': 'slider',
            }],
            namespace = 'target',
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
                '-d', os.path.join(get_package_share_directory('slider_experiment'), 'rviz_config/slider.rviz'),
            ],
        )
    )
    
    
    return ld

