from setuptools import setup
import os
from glob import glob

package_name = 'control'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Fourlas',
    maintainer_email='frank.fourlas@gmail.com',
    description='A package containing various controllers for the chasers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster = ' + package_name + '.thruster:main',
            'key_teleop = ' + package_name + '.key_teleop:main',
            'pid = ' + package_name + '.pid_controller:main',
            'thruster_pwm = ' + package_name + '.thruster_pwm:main',
            'mpc = ' + package_name + '.mpc_controller:main',
            'pose_match = ' + package_name + '.pose_match_mpc_controller:main',
        ],
    },
)
