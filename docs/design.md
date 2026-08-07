# Design Notes

## Goal

Develop a complete ROS 2 ecosystem for the Small Hammer 6-DOF robotic arm.

## Planned Features

- Robot description (URDF/Xacro)
- RViz visualization
- robot_state_publisher
- joint_state_publisher
- ESP32 hardware interface
- MoveIt 2 integration
- NVIDIA Isaac Sim support
- Digital Twin

## Repository Structure

- firmware/ - ESP32 firmware
- ros2_ws/ - ROS 2 workspace
- docs/ - Documentation
- cad/ - CAD files
- simulation/ - Simulation assets

## Current Status

- [x] Robot measured
- [x] Joint directions identified
- [ ] Build URDF/Xacro
- [ ] RViz visualization
- [ ] ROS 2 control
- [ ] MoveIt 2
- [ ] Isaac Sim