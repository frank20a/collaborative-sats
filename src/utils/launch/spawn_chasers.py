from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

def generate_launch_description():

    # raw_target = xacro.process_file('/home/frank20a/dev-ws/data/models/simple_targets/marker_cube.urdf.xacro').toxml()
    raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/chaser/chaser.urdf.xacro').toxml()
    world = os.path.join('/home/frank20a/dev-ws', 'worlds', 'space.world')
    
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
            output = 'screen',
            parameters = [{
                'robot_description': raw_chaser,
                'use_sim_time': True    # Use simulation ()
                }],
            remappings=[
                ('robot_description', 'chaser_desc')
            ]
        )
    )

    # Open RViz
    ld.add_entity(
        Node(
            package='rviz2',
            executable='rviz2',
            parameters = [{
                'use_sim_time': True
            }],
            arguments=[
                '-d', os.path.join(get_package_share_directory('pose_estimation'), 'rviz_config/space_sim.rviz')
            ]
        )
    )

    # Spawn chaser into Gazebo
    ld.add_entity(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawner',
            parameters = [{
                'use_sim_time': True
            }],
            arguments=[
                '-topic', '/chaser_desc', 
                '-entity', 'chaser',
                '-robot_namespace', 'chaser_1'
            ]
        )
    )

    # Undistorter
    ld.add_entity(
        Node(
            package='pose_estimation',
            executable='undistort',
            name='undistorter',
            parameters = [{
                'use_sim_time': True,
                # 'verbose': 1,
                'sim': True
            }],
        ),
    )

    return ld