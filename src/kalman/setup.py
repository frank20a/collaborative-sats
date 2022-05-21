from setuptools import setup

package_name = 'kalman'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Fourlas',
    maintainer_email='frank.fourlas@gmail.com',
    description='This package provides a Kalman filters for rigid body tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_rigid_body = ' + package_name + '.filter_rigid_body:main',
            'tester = ' + package_name + '.test:main',
        ],
    },
)
