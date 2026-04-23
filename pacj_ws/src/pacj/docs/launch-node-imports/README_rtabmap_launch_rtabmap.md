# `rtabmap_launch`: `rtabmap.launch.py`

## What this is

PACJ loads this file via `get_package_share_directory('rtabmap_launch')` and `IncludeLaunchDescription` twice in `launch/laptop.launch.py` (drone stack and rover stack) with different `namespace`, `rgb_topic`, `depth_topic`, `camera_info_topic`, `odom_topic`, frame IDs, database paths, and optional-sensor topics so each robot can run **its own** SLAM graph.

Upstream reference: [introlab/rtabmap_ros `rtabmap.launch.py` (ros2 branch)](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py).

Only nodes whose **launch conditions** evaluate true are started. The table below lists everything this file *can* start; the last section summarizes **PACJ’s current `laptop.launch.py` settings**.

## Nodes generated (all conditional branches)

Unless noted, nodes are placed under the launch argument `namespace` (PACJ uses `drone` or `rover`), so graph names look like `/drone/rtabmap`, `/rover/rgbd_odometry`, etc.

| Executable / `name` | Package | Brief role |
|---------------------|---------|--------------|
| `republish` (`republish_rgb`) | `image_transport` | Decompresses RGB from `.../compressed` (or similar) to raw for downstream nodes when `compressed==true` and RGB-D stereo mode is off. |
| `republish` (`republish_depth`) | `image_transport` | Same idea for depth (`compressedDepth` → raw) when `compressed==true`. |
| `rgbd_sync` | `rtabmap_sync` | Time-synchronizes separate RGB, depth, and `camera_info` into an internal RGB-D stream when `rgbd_sync==true` and not stereo. |
| `republish_left` / `republish_right` | `image_transport` | Stereo compressed relay when `stereo==true` and `compressed==true`. |
| `stereo_sync` | `rtabmap_sync` | Synchronizes left/right stereo images when `stereo==true` and `rgbd_sync==true`. |
| `rgbd_relay` | `rtabmap_util` | Relays `rgbd_image` when subscribing to pre-packed RGB-D and not using `rgbd_sync` (uncompressed path). |
| `rgbd_relay_uncompress` | `rtabmap_util` | Same family with `uncompress` when using compressed RGB-D topic without `rgbd_sync`. |
| `rgbd_odometry` | `rtabmap_odom` | **Visual RGB-D odometry**: estimates motion from aligned color + depth (+ optional IMU), publishes `odom` and TF when enabled. Default when `visual_odometry==true`, `icp_odometry==false`, `stereo==false`. |
| `stereo_odometry` | `rtabmap_odom` | Visual odometry from a stereo pair instead of RGB-D. |
| `icp_odometry` | `rtabmap_odom` | Scan / point-cloud odometry when `icp_odometry==true`. |
| `rtabmap` | `rtabmap_slam` | **Core SLAM / localization**: map optimization, loop closure, occupancy or cloud outputs, map TF, database I/O. |
| `rtabmap_viz` | `rtabmap_viz` | Optional RTAB-Map GUI for debugging and visualization. |
| `rviz2` | `rviz2` | Optional RViz started by this launch when `rviz==true`. |
| `point_cloud_xyzrgb` | `rtabmap_util` | Optional colored cloud helper when `rviz==true`. |

## PACJ `laptop.launch.py` subset (typical)

With the arguments currently passed in the repo (including `compressed:=false`, `rtabmap_viz:=false`, default `stereo:=false`, `icp_odometry:=false`, `rgbd_sync` default `false`, `rviz:=false`), you normally get **only**:

| Node | Role |
|------|------|
| `rgbd_odometry` | RGB-D visual odometry for that namespace. |
| `rtabmap` | SLAM / mapping for that namespace. |

If you change launch arguments (e.g. enable `compressed`, `rgbd_sync`, `rtabmap_viz`, or `rviz`), additional rows from the big table above will appear in `rqt_graph`.

## IMU / GPS / tags defaults

Upstream defaults remap optional inputs to **global** topic names (for example `/imu/data`, `/gps/fix`) unless you override them. PACJ overrides many of these per robot in `laptop.launch.py`; keep them aligned with what actually publishes on your network to avoid cross-robot subscriptions.
