from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
            'color_fps': '6',
            'depth_width': '480',
            'depth_height': '270',
            'depth_fps': '6',
            'color_format': 'MJPEG',          # High compression to save USB bandwidth
            'enable_point_cloud': 'false',    # Let RTAB-Map handle the 3D math instead
            # --------------------------------
        }.items()
    )

    pi_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='pi_camera',
        parameters=[{
            'image_size': [1920, 1080],
        }]
    )

    offboard_controller = Node(
        package='pacj',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
        parameters=[{
            'marker_size': 0.046
        }],
        remappings=[
            ('/camera_info', '/camera/camera_info'),
            ('/image_raw', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        # offboard_controller,
        # pi_camera,
        # (5s Delay)
        TimerAction(
            period=5.0,
            actions=[drone_camera]
        ),
    ])
