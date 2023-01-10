from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

def generate_launch_description():

    raw_chaser = xacro.process_file('/home/frank20a/dev-ws/data/models/chaser/chaser.urdf.xacro').toxml()

    world = os.path.join('/home/frank20a/dev-ws/data', 'worlds', 'calibration.world')
    

    return LaunchDescription([ 
    
        # Open Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items()
        ),
        
        # Publish chaser to robot_state_publisher
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
        ),

        # Spawn chaser into Gazebo
        

        Node(
            package = 'utils',
            executable = 'spawn_model',
            parameters = [{
                'name': 'chaser',
                'suffix': '0',
                'initial_pose': '{position: {x: 0.0, y:  0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0,  w:  1.0}}'
            }]
        ),

        # Start the calibration process
        Node(
            package='pose_estimation',
            executable='sim_calibrator',
            name='calibrator',
            parameters = [{
                'use_sim_time': True,
            }],
            remappings=[
                ('/sim_camera/image_raw', '/chaser_0/front_cam/image_raw'),
            ]
        ),

        
    ])