from click import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os, sys, xacro
# from control.control.pose_match_mpc_controller import nc as nc_


raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/chaser/chaser.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
world = os.path.join('/home/frank20a/dev-ws/data', 'worlds', 'space.world')

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
    ld.add_entity(
        Node(
            package = 'control',
            executable = 'pose_match',
            parameters=[{
                'nc': nc,
                'verbose': 2,
            }]
        )
    )

    for i in range(nc):
        ns = 'chaser_' + str(i)

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
                    'name': 'chaser',
                    'suffix': str(i),
                    'initial_pose': (
                        '{position: {x: -1.0, y:  0.5, z: 0.75}, orientation: {x: 0.0, y: 0.0, z: -0.258819, w: 0.9659258}}',
                        '{position: {x: -1.0, y: -0.5, z: 0.75}, orientation: {x: 0.0, y: 0.0, z:  0.258819, w: 0.9659258}}'
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
                    'use_sim_time': False
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
                namespace = 'chaser_' + str(i),
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
                '-d', os.path.join(get_package_share_directory('scenarios'), 'rviz_config/scenarios2.rviz'),
            ],
        )
    )
    
    
    return ld

