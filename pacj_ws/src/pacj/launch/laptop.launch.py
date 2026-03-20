from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')

    # decompressor = Node(
    #     package='pacj',
    #     executable='decompressor',
    #     name='decompressor',
    #     output='screen'
    # )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/rover/color/image_raw',
            'depth_topic': '/rover/depth/image_raw',
            'camera_info_topic': '/rover/color/camera_info',
            'frame_id': 'rover_link',          # This MUST match the camera's base frame
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'qos_camera': '1',
            'use_sim_time': 'false',
            'args': '--delete_db_on_start --Vis/MaxFeatures 600',
            'rtabmap_viz': 'false',
        }.items()
    )

    return LaunchDescription([
        # decompressor,
        slam
    ])