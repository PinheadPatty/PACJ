from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    orbbec_pkg = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg, 'launch', 'gemini_330_series.launch.py')

    drone_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments={
            'camera_name': 'drone',
            'enable_sync_output_accel_gyro': 'true',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'depth_registration': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480'
        }.items()
    )

    drone_driver = Node(
        package='pacj',
        executable='drone_driver',
        name='drone_driver',
        output='screen'
    )

    pi_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='pi_camera',
        parameters=[{
            'image_size': [1920, 1080],
        }]
    )

    aruco_node = Node(
        package='pacj',
        executable='aruco_landing_target',
        name='aruco_landing_target',
        output='screen',
        parameters=[{
            'marker_size': 0.046
        }],
        remappings=[
            ('/camera/image_raw', '/image_raw'),
            ('/camera/camera_info', '/camera_info')
        ]
    )

    return LaunchDescription([
        drone_camera,
        drone_driver,
        pi_camera,
        aruco_node,
    ])