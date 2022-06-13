from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

def generate_launch_description():

    raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/chaser/chaser.urdf.xacro').toxml()
    raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
    world = os.path.join('/home/frank20a/dev-ws/data', 'worlds', 'space.world')
    ld = LaunchDescription()

    # Open Gazebo
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items()
        )
    )

    # Publish chaser to robot_state_publisher
    ld.add_entity(
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            name = 'state_publisher',
            parameters = [{
                'robot_description': raw_chaser,
                'use_sim_time': True    # Use simulation ()
                }],
            remappings = [
                ('robot_description', 'chaser_desc'),
                # ('__ns', 'chaser_1'),
            ]
        )
    )

    # Publish target to robot_state_publisher
    ld.add_entity(
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            name = 'state_publisher',
            output = 'screen',
            parameters = [{
                'robot_description': raw_target,
                'use_sim_time': True    # Use simulation ()
                }],
            remappings=[
                ('robot_description', 'target_desc'),
            ]
        )
    )

    # Spawn chaser into Gazebo
    ld.add_entity(
        Node(
            package = 'utils',
            executable = 'spawn_model',
            name = 'spawner',
            parameters = [{
                'name': 'chaser',
                # 'suffix': '1',
                'initial_pose': '{position: {x: 0.0, y: 0.0, z: 0.85}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
            }]
        )
    )

    # Open Undistorter
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable = 'undistort',
            parameters = [{
                'use_sim_time': True,
                # 'verbose': 1,
                'sim': True
            }],
            remappings = [
                # ('__ns', 'chaser_1'),
            ]
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
            remappings = [
                # ('__ns', 'chaser_1'),
            ]
        )
    )

    # Open ArUco estimator
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable = 'aruco_board_estimator',
            parameters = [{
                # 'verbose': 3,
                'use_sim_time': True,
                'sim': True
            }],
            remappings = [
                # ('__ns', 'chaser_1'),
            ]
        )
    )

    # Open RViz
    ld.add_entity(
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            output = {'both': 'log'},
            parameters = [{
                'use_sim_time': True
            }],
            arguments = [
                '-d', os.path.join(get_package_share_directory('pose_estimation'), 'rviz_config/aruco.rviz'),
            ],
        )
    )
    
    return ld

