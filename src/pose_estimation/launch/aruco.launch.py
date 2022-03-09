from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro, os

def generate_launch_description():

    return LaunchDescription([
        # Open RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', os.path.join(get_package_share_directory('pose_estimation'), 'rviz_config/aruco.rviz')
            ]
        ),
        
        # Open Undistorter
        Node(
            package='pose_estimation',
            executable='undistort'
        ),
        
        # Open ArUco estimator
        Node(
            package='pose_estimation',
            executable='aruco_pose_estimation',
            arguments=[
                '-p', 'marker_size:=0.13'
            ]
        ),
    ])