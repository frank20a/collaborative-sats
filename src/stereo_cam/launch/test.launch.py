from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro, os

def generate_launch_description():

    raw_robot = xacro.process_file('/home/frank20a/dev-ws/data/models/test/test.urdf.xacro').toxml()

    return LaunchDescription([
        Node(
            package = 'robot_state_publisher',
            executable =  'robot_state_publisher',
            name = 'state_publisher',
            parameters = [{'robot_description': raw_robot}]
        ),
        Node(
            package = 'joint_state_publisher_gui',
            executable = 'joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])