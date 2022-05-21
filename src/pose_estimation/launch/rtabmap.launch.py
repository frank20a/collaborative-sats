from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os


raw_chaser = xacro.process_file('/home/frank20a/dev-ws/models/chaser/chaser.urdf.xacro').toxml()
raw_target = xacro.process_file('/home/frank20a/dev-ws/models/simple_targets/marker_cube.urdf.xacro').toxml()
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
    
    # Start Stereo Image Processor
    ld.add_entity(
        Node(
            package = 'stereo_image_proc',
            executable = 'stereo_image_proc',
            name = 'stereo_image_proc',
            namespace='stereo_camera',
        )
    )

    # Start Stereo Odometry
    ld.add_entity(
        Node(
            package = 'rtabmap_ros',
            executable = 'stereo_odometry',
            name = 'stereo_odometry',
            output = 'screen',
            remappings = [
                ('left/image_rect', 'left/image_rect'),
                ('right/image_rect', 'right/image_rect'),
                ('left/camera_info', 'left/camera_info'),
                ('right/camera_info', 'right/camera_info'),
            ],
            parameters = [{
                'frame_id': 'chaser_0/camera_optical',
                'odom_frame_id': 'target/body',
                'approx_sync': True,
                'queue_size': 5,
                'Odom/MinInliers': 12,
                'Odom/RoiRatios': '0.03 0.03 0.04 0.04'
            }],
            namespace='stereo_camera'
        )
    )

    # Start Disparity to depth
    ld.add_entity(
        Node(
            package = 'rtabmap_ros',
            executable = 'rtabmap',
            name = 'rtabmap',
            output = 'screen',
            arguments = [
                '--delete_db_on_start',
            ],
            remappings = [
                ('left/image_rect', 'stereo_camera/left/image_rect_color'),
                ('right/image_rect', 'stereo_camera/right/image_rect'),
                ('left/camera_info', 'stereo_camera/left/camera_info'),
                ('right/camera_info', 'stereo_camera/right/camera_info'),
                ('odom', 'stereo_camera/odom'),
            ],
            parameters = [{
                'frame_id': 'chaser_0/camera_optical',
                'subscribe_stereo': True,
                'subscribe_depth': False,
                'approx_sync': True,
                'queue_size': 30,
                'Vis/MinInliers': '12',
            }],
            namespace='stereo_camera'
        )
    )

    # Open RTAB-Map-Viz
    ld.add_entity(
        Node(
            package = 'rtabmap_ros',
            executable = 'rtabmapviz',
            name = 'rtabmapviz',
            output = 'screen',
            arguments = [
                '-d', '$(find rtabmap_ros)/launch/config/rgbd_gui.ini'
            ],
            remappings = [
                ('left/image_rect', 'stereo_camera/left/image_rect_color'),
                ('right/image_rect', 'stereo_camera/right/image_rect'),
                ('left/camera_info', 'stereo_camera/left/camera_info'),
                ('right/camera_info', 'stereo_camera/right/camera_info'),
                ('odom_info', 'stereo_camera/odom_info'),
                ('odom', 'stereo_camera/odom'),
            ],
            parameters = [{
                'frame_id': 'chaser_0/camera_optical',
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'queue_size': 10,
            }],
            namespace='stereo_camera'
        )
    )

    
    return ld

