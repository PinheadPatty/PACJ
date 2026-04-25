from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pacj'

_calibration_dir = 'calibration'
_calibration_files = (
    sorted(glob(os.path.join(_calibration_dir, '*')))
    if os.path.isdir(_calibration_dir)
    else []
)

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
    ]
    + (
        [(os.path.join('share', package_name, 'calibration'), _calibration_files)]
        if _calibration_files
        else []
    ),
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
            'battery = pacj.battery:main',
            'interactive_setpoint = pacj.interactive_setpoint:main',
            'drone_planner = pacj.drone_planner:main',
            'offboard_controller = pacj.offboard_controller:main',
            'aruco_detector = pacj.aruco_detector_node:main',
            'charuco_capture = pacj.charuco_capture_node:main',
            'charuco_calibrate_from_images = pacj.charuco_calibrate_from_images_node:main',
        ],
    },
)
