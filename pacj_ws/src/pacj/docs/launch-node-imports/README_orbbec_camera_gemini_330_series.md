# `orbbec_camera`: `gemini_330_series.launch.py`

## What this is

PACJ loads this file via `get_package_share_directory('orbbec_camera')` and `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` so a **Gemini 330-series** depth camera publishes color, depth, camera calibration, optional IMU-style streams, and related TF. The same launch is used on the drone (`camera_name:=drone`) and rover (`camera_name:=rover`) with different namespaces so topics stay separated.

Upstream source (reference): [OrbbecSDK_ROS2 `gemini_330_series.launch.py`](https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/orbbec_camera/launch/gemini_330_series.launch.py).

## ROS distro behavior

- **ROS 2 Foxy**: starts a standalone node `ob_camera_node` (legacy path in upstream; most teams use newer distros).
- **Humble and newer** (typical for PACJ): starts a **component container** and loads the Orbbec driver as a **composable node** inside it, under the launch argument `camera_name` (pushed as a ROS namespace).

## Nodes generated (Humble+ layout)

Names below use your `camera_name` (e.g. `drone` or `rover`). Resolved graph names are usually under `/<camera_name>/...`.

| Node / graph name (pattern) | Brief role |
|-----------------------------|------------|
| `/<camera_name>/camera_container` | `rclcpp_components` **component_container** process. Hosts composable plugins in one process. |
| `/<camera_name>/<camera_name>` (composable) | **OBCameraNodeDriver** plugin: opens the USB device, publishes color/depth streams, `camera_info`, optional point clouds, diagnostics, and TF when enabled. This is the actual camera driver logic. |

If your `rqt_graph` shows both a `camera_container` and a named driver node under the same namespace, that is expected for composable launches: one container, one loaded driver component.

## What PACJ passes

See `launch/drone.launch.py` and `launch/rover.launch.py` for the `launch_arguments` block (resolution, sync, MJPEG color, reduced resolution/FPS, `enable_point_cloud`, etc.).
