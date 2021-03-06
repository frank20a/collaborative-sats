from setuptools import setup
import os
from glob import glob

package_name = 'pose_estimation'

setup(
    name=package_name,
    version='0.3.1',
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
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrator = ' + package_name + '.calibrate:main',
            'sim_calibrator = ' + package_name + '.calibrate_sim:main',
            'undistort = ' + package_name + '.undistort:main',
            'aruco_estimator = ' + package_name + '.aruco:main',
            'aruco_board_estimator = ' + package_name + '.aruco_board:main',
            'combine_estimations = ' + package_name + '.combine_estimations:main',
        ],
    },
)
