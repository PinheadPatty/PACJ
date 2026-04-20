from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')

    # --- YOUR ORIGINAL NODES (restored) ---
    vio_relay = Node(
        package='pacj',
        executable='vio_relay',
        name='vio_relay',
        output='screen'
    )

    tf_broadcaster = Node(
        package='pacj',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen'
    )

    interactive_setpoint = Node(
        package='pacj',
        executable='interactive_setpoint',
        name='interactive_setpoint',
        output='screen'
    )

    drone_planner = Node(
        package='pacj',
        executable='drone_planner',
        name='drone_planner',
        output='screen'
    )

    # --- SLAM ---
    drone_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/drone/color/image_raw',
            'depth_topic': '/drone/depth/image_raw',
            'camera_info_topic': '/drone/color/camera_info',
            'namespace': 'drone',
            'odom_topic': '/drone/odom',
            'frame_id': 'drone_link',
            'odom_frame_id': 'drone/odom',
            'map_frame_id': 'drone/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'qos_camera': '1',
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
            'odom_topic': '/rover/odom',
            'frame_id': 'rover_link',
            'odom_frame_id': 'rover/odom',
            'map_frame_id': 'rover/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'qos_camera': '1',
            'use_sim_time': 'false',
            'args': '--delete_db_on_start --Vis/MaxFeatures 600 --database_path /tmp/rover_rtabmap.db',
            'rtabmap_viz': 'false',
            'Grid/3D': 'true',
            'publish_tf_odom': 'true',
        }.items()
    )

    # --- RVIZ CONFIGS ---
    pkg_share = get_package_share_directory('pacj')

    drone_rviz_config = os.path.join(pkg_share, 'rviz', 'drone_config.rviz')
    rover_rviz_config = os.path.join(pkg_share, 'rviz', 'rover_config.rviz')

    # --- TWO RVIZ INSTANCES ---
    rviz_drone = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_drone',
        arguments=['-d', drone_rviz_config],
        output='screen'
    )

    rviz_rover = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_rover',
        arguments=['-d', rover_rviz_config],
        output='screen'
    )

    return LaunchDescription([
        # vio_relay,
        # tf_broadcaster,
        # interactive_setpoint,
        # drone_planner,
        # drone_slam,
        rover_slam,
        rviz_drone,
        rviz_rover
    ])