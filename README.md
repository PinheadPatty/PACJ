# PACJ

PACJ is a ROS 2 workspace for autonomous rover + drone workflows, including:

- Offboard drone control
- Mapping and localization
- ArUco-assisted landing and docking logic
- Rover module grabbing and placement

## Repository Layout

- `pacj_ws/` - ROS 2 workspace and packages
- `pacj_ws/src/pacj/` - main PACJ nodes, launch files, and configs
- `pacj_ws/src/px4_msgs/` - PX4 ROS message submodule

## Included Dependencies

- Orbbec ROS 2 SDK wrapper: [Orbbec G330 ROS 2 Manual](https://www.orbbec.com/docs/g330-ros-2-wrapper-user-manual/#installation-instructions)

## GitHub Pages

If GitHub Pages is enabled for this repo, view it at:

- [https://pinheadpatty.github.io/PACJ/](https://pinheadpatty.github.io/PACJ/)