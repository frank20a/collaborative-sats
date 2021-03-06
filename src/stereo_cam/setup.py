from setuptools import setup
import os
from glob import glob

package_name = 'stereo_cam'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Fetch stereoscopic data from sensors and process them',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_viewer = ' + package_name + '.stereo_viewer:main',
            'disparity_viewer = ' + package_name + '.disparity_viewer:main',
            'disparity_publisher = ' + package_name + '.disparity_publisher:main',
            'pcd_publisher = ' + package_name + '.pcd_publisher:main'
        ],
    },
)
