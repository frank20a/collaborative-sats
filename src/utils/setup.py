from struct import pack
from setuptools import setup
from glob import glob
import os

package_name = 'utils'

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
    description='Utillities for other packages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_model = ' + package_name + '.spawn_model:main',
            'odometry2tf = ' + package_name + '.odometry2tf:main',
            'tf_debugger = ' + package_name + '.tf_debugger:main',
            'setpoint_publisher = ' + package_name + '.setpoint_publisher:main',
            '8bit_flag_tester = ' + package_name + '.8bit_flag_tester:main',
            'camera_publisher = ' + package_name + '.camera_publisher:main',
        ],
    },
)
