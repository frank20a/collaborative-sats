from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

def generate_launch_description():

    raw_robot = xacro.process_file('/home/frank20a/dev-ws/models/simple_targets/cubesat.urdf.xacro').toxml()

    
    return LaunchDescription([
        # Open Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        ),

        # Publish robot to robot_state_publisher
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            name = 'state_publisher',
            output = 'screen',
            parameters = [{
                'robot_description': raw_robot,
                'use_sim_time': True    # Use simulation ()
                }],
            remappings=[
                ('robot_description', 'easy_cube_desc')
            ]
        ),

        # Open RViz
        Node(
            package='rviz2',
            executable='rviz2',
            parameters = [{
                'use_sim_time': True
            }],
            # arguments=[
            #     '-d', os.path.join(get_package_share_directory('stereo_cam'), 'rviz_config/stereo_cam.rviz')
            # ]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawner',
            arguments=[
                '-topic', 'easy_cube_desc', 
                '-entity', 'stereo'
                ]
        ),

        # Spawn a disparity publisher
        # Node(
        #     package='stereo_cam',
        #     executable='disparity_publisher',
        #     name='disparity_publisher'
        # )
    ])