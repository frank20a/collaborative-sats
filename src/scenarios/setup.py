from setuptools import setup
import os
from glob import glob

package_name = 'scenarios'

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
    maintainer='Frank Fourlas',
    maintainer_email='frank.fourlas@gmail.com',
    description='Different control scenarios to test the control method',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lock_target = ' + package_name + '.lock_target:main',
            'pose_match = ' + package_name + '.pose_match:main',
        ],
    },
)
