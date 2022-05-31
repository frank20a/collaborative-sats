from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import xacro, os, sys

raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/slider/slider.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
world = os.path.join('/home/frank20a/dev-ws', 'worlds', 'lab.world')
nc = 1

nc = None
for arg in sys.argv:
    if arg.startswith("nc"):
        nc = int(arg.split(":=")[1])

if nc is None:
    nc = 1


def generate_launch_description():

    ld = LaunchDescription()    
    
    # ================= Simulation =================
    # Open Gazebo
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items()
        )
    )

    # Publish target to robot_state_publisher
    ld.add_entity(
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            name = 'target_state_publisher',
            output = 'screen',
            parameters = [{
                'robot_description': raw_target,
                'use_sim_time': True,
            }],
            remappings=[
                ('robot_description', 'target_desc'),
            ],
            namespace = 'target',
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
                'verbose': 2
            }]
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

        # Spawn chaser into Gazebo
        ld.add_entity(
            Node(
                package = 'utils',
                executable = 'spawn_model',
                parameters = [{
                    'name': 'slider',
                    'suffix': str(i),
                    'initial_pose': (
                        '{position: {x: -1.0, y:  0.5, z: 0.1185}, orientation: {x: 0.0, y: 0.0, z: -0.258819, w: 0.9659258}}',
                        '{position: {x: -1.0, y: -0.5, z: 0.1185}, orientation: {x: 0.0, y: 0.0, z:  0.258819, w: 0.9659258}}'
                    )[i]
                }]
            )
        )

        # Start Odometry translator
        ld.add_entity(
            Node(
                package = 'utils',
                executable = 'odometry2tf',
                parameters = [{
                    'use_sim_time': True
                }],
                namespace = ns
            )
        )
        
        # Open ArUco estimator
        ld.add_entity(
            Node(
                package = 'pose_estimation',
                executable = 'aruco_board_estimator',
                parameters = [{
                    # 'verbose': 1,
                    'use_sim_time': True,
                    'sim': True,
                    'filter': 'const_accel',
                    'duration': False,
                    'ra_len': 5,
                }],
                remappings=[
                    ('input_img', 'front_cam/image_raw'),
                ],
                namespace = ns
            )
        )

        # Controller
        ld.add_entity(
            Node(
                package = 'control',
                executable = 'thruster_pwm',
                parameters = [{
                    'force': 1.2,
                    'torque': 0.54,
                }],
                namespace = ns,
            ),
        )

    # Estimation combiner
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable = 'combine_estimations',
            parameters =[{
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

