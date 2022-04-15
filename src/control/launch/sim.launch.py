from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
import xacro, os


raw_chaser = xacro.process_file('/home/frank20a/dev-ws/models/chaser/chaser.urdf.xacro').toxml()
world = os.path.join('/home/frank20a/dev-ws', 'worlds', 'space.world')


def generate_launch_description():

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
            output = {'both': 'log'},
            parameters = [{
                'robot_description': raw_chaser,
                'use_sim_time': True,
                'frame_prefix': 'chaser_0/'
            }],
            namespace = 'chaser_0'
        )
    )
    
    # Spawn chaser into Gazebo
    ld.add_entity(
        Node(
            package = 'utils',
            executable = 'spawn_model',
            output = {'both': 'log'},
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
            output = {'both': 'log'},
            parameters = [{
                'use_sim_time': True
            }],
            namespace = 'chaser_0'
        )
    )
    
    # Start plotjuggler
    ld.add_entity(
        ExecuteProcess(
            cmd = ['ros2', 'run', 'plotjuggler', 'plotjuggler', '--layout', '/home/frank20a/dev-ws/data/plotjuggler-configs/pid.xml'],
            name = 'plotjuggler',
            output = {'both': 'log'},
        )
    )
    
    return ld

