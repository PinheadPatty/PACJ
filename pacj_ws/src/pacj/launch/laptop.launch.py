from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')

    decompressor = Node(
        package='pacj',
        executable='decompressor',
        name='decompressor',
        output='screen'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/drone/color/image_raw',
            'depth_topic': '/drone/depth/image_raw',
            'camera_info_topic': '/drone/color/camera_info',
            'frame_id': 'drone_color_optical_frame',
            'approx_sync': 'true',
            'args': '-d --DeleteDbOnStart' 
        }.items()
    )

    return LaunchDescription([
        decompressor,
        slam
    ])