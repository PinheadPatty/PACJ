from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pacj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pacj',
    maintainer_email='pacj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'decompressor = pacj.decompress:main',
            'rover_driver = pacj.rover_driver:main',
            'drone_driver = pacj.drone_driver:main',
            'battery = pacj.battery:main',
            'vio_relay = pacj.vio_relay:main',
            'tf_broadcaster = pacj.tf_broadcaster:main',
            'interactive_setpoint = pacj.interactive_setpoint:main',
            'drone_planner = pacj.drone_planner:main',
            'camera_rgb_stream = pacj.camera_rgb_stream_node:main',
        ],
    },
)
