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
    # # Vicon Receiver
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('/home/frank20a/dev-ws/data/models/gazebo-slider/example/example.launch.py'),
        )
    )

    # # Vicon Translator
    # ld.add_entity(
    #     Node(
    #         package = 'slider_experiment',
    #         executable = 'vicon_filter',
    #         remappings=[
    #             ('input', 'vicon/slider_0605/slider_0605'),
    #         ],
    #         parameters = [{
    #             'master': True,
    #         }]
    #     )
    # )

    # Publish target to robot_state_publisher
    # ld.add_entity(
    #     Node(
    #         package = 'robot_state_publisher',
    #         executable =  'robot_state_publisher',
    #         name = 'target_state_publisher',
    #         output = 'screen',
    #         parameters = [{
    #             'robot_description': raw_target,
    #             'use_sim_time': True,
    #         }],
    #         remappings=[
    #             ('robot_description', 'target_desc'),
    #         ],
    #         namespace = 'target',
    #     )
    # )

    # ================= Chasers =================
    # Controller
    ld.add_entity(
        Node(
            package = 'slider_experiment',
            executable = 'dummy_controller',
            parameters=[{
                'nc': nc,
                'verbose': 2
            }],
            remappings=[
                ('slider_0/odom', 'slider/odom')
            ]
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
                    'frequency': 10,
                    'resolution': 16
                }],
            remappings=[
                ('slider_0/odom', 'slider/odom')
            ]
            ),
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

