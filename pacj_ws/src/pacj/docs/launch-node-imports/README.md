# Launch imports: generated ROS nodes

This folder documents **external** launch files that PACJ includes with `IncludeLaunchDescription`. Those files start their own executables; they are not defined inline in `drone.launch.py`, `rover.launch.py`, or `laptop.launch.py`.

| Document | Upstream package / launch | Included from |
|----------|---------------------------|---------------|
| [README_orbbec_camera_gemini_330_series.md](README_orbbec_camera_gemini_330_series.md) | `orbbec_camera` → `gemini_330_series.launch.py` | `launch/drone.launch.py`, `launch/rover.launch.py` |
| [README_rtabmap_launch_rtabmap.md](README_rtabmap_launch_rtabmap.md) | `rtabmap_launch` → `rtabmap.launch.py` | `launch/laptop.launch.py` |

If you add another `IncludeLaunchDescription` that starts nodes, add a README here and a row to the table.
