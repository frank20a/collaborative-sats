from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

raw_chaser = xacro.process_file('/home/frank20a/dev-ws/models/chaser/chaser.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/models/simple_targets/marker_cube.urdf.xacro').toxml()
world = os.path.join('/home/frank20a/dev-ws', 'worlds', 'space.world')
ns = 'chaser_0'

def generate_launch_description():

    ld = LaunchDescription()

# ================= Gazebo =================
    # Open Gazebo
    ld.add_entity(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items()
        )
    )

# ================= Target =================
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
                'frame_prefix': ns + '/',
            }],
            remappings=[
                ('robot_description', 'target_desc'),
            ],
            namespace = 'target',
        )
    )

# ================= Chaser =================
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
                'suffix': str(0),
                'initial_pose': '{position: {x: -1.0, y: 0.5, z: 0.75}, orientation: {x: 0.0, y: 0.0, z: -0.258819, w: 0.9659258}}'
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
                'duration': False
            }],
            remappings=[
                ('input_img', 'front_cam/image_raw'),
            ],
            namespace = ns
        )
    )

# ================= RViz =================
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
                '-d', os.path.join(get_package_share_directory('scenarios'), 'rviz_config/scenarios1.rviz'),
            ],
        )
    )

    return ld
