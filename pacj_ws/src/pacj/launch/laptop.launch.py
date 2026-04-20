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
    # Ground truth for who subscribes to what: ros2 topic info -v /rover/color/image_raw
    # (Subscribers must list /rover/rgbd_odometry only — not rqt_graph alone).
    # rtabmap.launch.py defaults wire imu/gps/tags to GLOBAL names (/imu/data, /gps/fix,
    # /detections, /user_data_async). With two robots on one DDS domain, both stacks would
    # subscribe to the same topics — wrong. Point optional sensors at per-robot names; if
    # nothing publishes there, those inputs are simply unused.
    drone_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/drone/color/image_raw',
            'depth_topic': '/drone/depth/image_raw',
            'camera_info_topic': '/drone/color/camera_info',
            'namespace': 'drone',
            'imu_topic': '/drone/imu/data',
            'gps_topic': '/drone/gps/fix',
            'tag_topic': '/drone/apriltag/detections',
            'user_data_topic': '/drone/user_data',
            'user_data_async_topic': '/drone/user_data_async',
            'fiducial_topic': '/drone/fiducial_transforms',
            'env_sensor_topic': '/drone/env_sensor',
            'odom_topic': '/drone/odom',
            'frame_id': 'drone_link',
            'odom_frame_id': 'drone/odom',
            'map_frame_id': 'drone/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            # Must match rgb/depth_image_transport: enables image_transport republish in rtabmap.launch.py
            'compressed': 'true',
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
            'imu_topic': '/rover/imu/data',
            'gps_topic': '/rover/gps/fix',
            'tag_topic': '/rover/apriltag/detections',
            'user_data_topic': '/rover/user_data',
            'user_data_async_topic': '/rover/user_data_async',
            'fiducial_topic': '/rover/fiducial_transforms',
            'env_sensor_topic': '/rover/env_sensor',
            'odom_topic': '/rover/odom',
            'frame_id': 'rover_link',
            'odom_frame_id': 'rover/odom',
            'map_frame_id': 'rover/map',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'compressed': 'true',
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
        # interactive_setpoint,
        # drone_planner,
        drone_slam,
        rover_slam,
        rviz_drone,
        rviz_rover
    ])