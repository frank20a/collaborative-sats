from setuptools import setup
import os
from glob import glob

package_name = 'pose_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'rviz_config'), 
            glob('rviz_config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Fourlas',
    maintainer_email='frank.fourlas@gmail.com',
    description='A package dedicated to estimating the pose of a target in various scenarios',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate = ' + package_name + '.calibrate:main',
            'undistort = ' + package_name + '.undistort:main',
            'aruco_pose_estimation = ' + package_name + '.aruco:main',
            'calibrate_sim = ' + package_name + '.calibrate_sim:main'
        ],
    },
)
