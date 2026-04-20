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
            'enable_sync': 'true',
            'depth_registration': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            # --- New "Lower CPU" Settings ---
            'color_width': '424',
            'color_height': '240',
            'color_fps': '10',
            'depth_width': '424',
            'depth_height': '240',
            'depth_fps': '10',
            'color_format': 'MJPEG',          # High compression to save USB bandwidth
            'enable_point_cloud': 'false',    # Let RTAB-Map handle the 3D math instead
            # --------------------------------
        }.items()
    )

    drone_driver = Node(
        package='pacj',
        executable='drone_driver',
        name='drone_driver',
        output='screen'
    )

    return LaunchDescription([
        drone_camera,
        drone_driver,
    ])