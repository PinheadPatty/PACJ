from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')

    drone_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/drone/camera/color/image_raw',
            'depth_topic': '/drone/camera/depth/image_raw',
            'camera_info_topic': '/drone/camera/color/camera_info',
            'namespace': 'drone',
            'frame_id': 'drone_link',
            'odom_frame_id': 'drone/odom',
            'map_frame_id': 'drone/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'compressed': 'false',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'use_sim_time': 'false',
            'args': '--delete_db_on_start --Vis/MaxFeatures 600 --database_path /tmp/drone_rtabmap.db',
            'rtabmap_viz': 'false',
            'Grid/3D': 'true',
            'publish_tf_odom': 'true',
        }.items()
    )

    rover_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/rover/color/image_raw',
            'depth_topic': '/rover/depth/image_raw',
            'camera_info_topic': '/rover/color/camera_info',
            'namespace': 'rover',
            'frame_id': 'rover_link',
            'odom_frame_id': 'rover/odom',
            'map_frame_id': 'rover/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'compressed': 'false',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'use_sim_time': 'false',
            'args': '--delete_db_on_start --Vis/MaxFeatures 600 --database_path /tmp/rover_rtabmap.db',
            'rtabmap_viz': 'false',
            'Grid/3D': 'true',
            'publish_tf_odom': 'true',
        }.items()
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            # Drop old WebSocket data when the browser can't keep up instead of
            # buffering it indefinitely (keeps the feed live, not historically accurate).
            'send_buffer_limit': 10000000,  # 10 MB
        }],
    )

    return LaunchDescription([
        drone_slam,
        rover_slam,
        foxglove_bridge,
    ])
