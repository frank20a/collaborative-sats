from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os


raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/chaser/chaser.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
world = os.path.join('/home/frank20a/dev-ws', 'worlds', 'space.world')


def robot_spawner(ld: LaunchDescription, robot_count: int):
    if robot_count > 1: return
    ns = 'chaser_' + str(robot_count)

    # Publish chaser to robot_state_publisher
    ld.add_entity(
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            parameters = [{
                'robot_description': raw_chaser,
                'use_sim_time': True,
                'frame_prefix': ns + '/'
            }],
            namespace = ns
        )
    )

    # Spawn chaser into Gazebo
    if robot_count == 0:
        pose = '{position: {x: -1.0, y: 0.5, z: 0.75}, orientation: {x: 0.0, y: 0.0, z: -0.258819, w: 0.9659258}}'
    else:
        pose = '{position: {x: -1.0, y: -0.5, z: 0.85}, orientation: {x: 0.0, y: 0.0, z: 0.258819, w: 0.9659258}}'
    ld.add_entity(
        Node(
            package = 'utils',
            executable = 'spawn_model',
            parameters = [{
                'name': 'chaser',
                'suffix': str(robot_count),
                'initial_pose': pose
            }]
        )
    )

    ## Open Undistorter
    # ld.add_entity(
    #     Node(
    #         package = 'pose_estimation',
    #         executable = 'undistort',
    #         parameters = [{
    #             'use_sim_time': True,
    #             'verbose': 0,
    #             'sim': True,
    #             'duration': False
    #         }],
    #         namespace = ns
    #     )        
    # )

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
                'duration': False
            }],
            remappings=[
                ('input_img', 'front_cam/image_raw'),
            ],
            namespace = ns
        )
    )


def generate_launch_description():

    ld = LaunchDescription()

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
                'use_sim_time': True    # Use simulation ()
                }],
            remappings=[
                ('robot_description', 'target_desc'),
            ]
        )
    )

    # Add chasers
    for i in range(2):
        robot_spawner(ld, i)

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
    
    # Open Estimation Combiner
    ld.add_entity(
        Node(
            package = 'pose_estimation',
            executable = 'combine_estimations',
            parameters =[{
                'duration': False,
                'use_sim_time': True
            }],
        )
    )
    
    return ld

