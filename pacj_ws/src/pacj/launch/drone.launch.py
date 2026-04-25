from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    orbbec_pkg = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg, 'launch', 'gemini_330_series.launch.py')

    uxrce_agent = ExecuteProcess(
        cmd=[
            '/usr/local/bin/MicroXRCEAgent',
            'serial',
            '--dev', '/dev/ttyAMA0',
            '-b', '921600',
        ],
        output='screen',
    )
    
    drone_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments={
            'camera_name': 'drone',
            'enable_sync': 'false',
            'depth_registration': 'false',
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
            'enable_laser': 'false',
            'laser_energy_level': '-1',
            'connection_delay': '500',
        }.items()
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='downward_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'frame_id': 'downward_camera_optical_frame',
            # 'camera': 0,  # or string id from rpicam-hello --list-cameras
        }],
        remappings=[
            ('image_raw', '/downward_camera/image_raw'),
            ('camera_info', '/downward_camera/camera_info'),
        ],
    )

    # Gated: publish_setpoints + Offboard; then publish_velocity / publish_position /
    # publish_landing for each control mode (all default false in the node).
    offboard_controller = Node(
        package='pacj',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
        parameters=[{
            'marker_size': 0.046,
            # 'publish_setpoints': True,
            # 'publish_velocity': True,
            # 'publish_position': True,
            # 'publish_landing': True,
        }],
        remappings=[
            ('/camera_info', '/camera/camera_info'),
            ('/image_raw', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        uxrce_agent,
        # offboard_controller,
        camera_node,
        # (5s Delay)
        TimerAction(
            period=5.0,
            actions=[drone_camera]
        ),
    ])
