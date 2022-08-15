from setuptools import setup
import os
from glob import glob

package_name = 'slider_experiment'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='frank20a',
    maintainer_email='frank.fourlas@gmail.com',
    description='Launch files & packages for the slider experiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slider_mpc = ' + package_name + '.slider_mpc_controller:main',
            'dummy_controller = ' + package_name + '.dummy_controller:main',
            'thruster_pwm = ' + package_name + '.thruster_pwm:main',
            'tsl = ' + package_name + '.thruster_pwm_tsl:main',
            'thruster_test = ' + package_name + '.thruster_test:main',
            'vicon_filter = ' + package_name + '.vicon_filter:main',
            'circle = ' + package_name + '.circle:main',
            'pose_raw_converter = ' + package_name + '.pose_raw_converter:main',
        ],
    },
)
