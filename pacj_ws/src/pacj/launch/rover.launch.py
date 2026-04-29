from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Rover driver node
    rover_driver_node = Node(
        package='pacj',
        executable='rover_driver',
        name='rover_driver',
        output='screen'
    )

    rover_battery_node = Node(
        package='pacj',
        executable='battery',
        name='battery_monitor',
        output='screen'
    )

    orbbec_pkg = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg, 'launch', 'gemini_330_series.launch.py')

    rover_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments={
            'camera_name': 'rover',
            'enable_sync': 'true',
            'depth_registration': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'color_width': '424',
            'color_height': '240',
            'color_fps': '6',
            'depth_width': '480',
            'depth_height': '270',
            'depth_fps': '6',
            'color_format': 'MJPEG',
            'enable_point_cloud': 'false',
        }.items()
    )

    return LaunchDescription([
        rover_driver_node,
        rover_battery_node,
        rover_camera_node,
    ])
