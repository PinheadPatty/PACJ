from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    orbbec_pkg = get_package_share_directory('orbbec_camera')
    orbbec_launch_path = os.path.join(orbbec_pkg, 'launch', 'gemini_330_series.launch.py')
    pkg_share = get_package_share_directory('pacj')
    urdf_path = os.path.join(pkg_share, 'description', 'rover.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'rover_controllers.yaml')
    drone_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments={
            'camera_name': 'rover',
            'enable_sync_output_accel_gyro': 'true',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'depth_registration': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480'
        }.items()
    )

    # Process XACRO and publish /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    # This is the main "Hardware Bridge" that talks to your U2D2
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'robot_description': Command(['xacro ', urdf_path])}],
        remappings=[('/robot_description', 'robot_description')],
        output='screen'
    )

    # This "spawner" starts the Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # This "spawner" starts the Diff Drive Controller for teleop
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    return LaunchDescription([
        drone_camera,
        robot_state_publisher_node,
        control_node,
	jsb_spawner,
	diff_drive_spawner
    ])
