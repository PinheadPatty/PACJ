from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_path = os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')

    # Ensure all custom nodes use simulation time
    sim_time_param = {'use_sim_time': True}

    vio_relay = Node(
        package='pacj',
        executable='vio_relay',
        name='vio_relay',
        output='screen',
        parameters=[sim_time_param]
    )

    tf_broadcaster = Node(
        package='pacj',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        parameters=[sim_time_param]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/drone/color/image_raw',
            'depth_topic': '/drone/depth/image_raw',
            'camera_info_topic': '/drone/color/camera_info',
            'frame_id': 'drone_link',          # This MUST match the camera's base frame
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',
            'rgb_image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'qos': '1',
            'qos_camera': '1',
            'use_sim_time': 'false',
            'args': '--delete_db_on_start --Vis/MaxFeatures 600',
            'rtabmap_viz': 'false',
            'Grid/3D': 'true', # Enable OctoMap Native 3D Grid
            'publish_tf_odom': 'false', # IMPORTANT: Let PX4 handle TF, not RTAB-Map
        }.items()
    )

    interactive_setpoint = Node(
        package='pacj',
        executable='interactive_setpoint',
        name='interactive_setpoint',
        output='screen',
        parameters=[sim_time_param]
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('pacj'),
        'rviz',
        'drone_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        parameters=[sim_time_param]
    )

    drone_planner = Node(
        package='pacj',
        executable='drone_planner',
        name='drone_planner',
        output='screen',
        parameters=[sim_time_param]
    )

    return LaunchDescription([
#        vio_relay,
#        tf_broadcaster,
#        interactive_setpoint,
#        drone_planner,
        slam,
        rviz_node
    ])
