# Architecture

This document describes the system and technical design for MarsRover-AutoNav-Blackout.

- Components: SensorFusion (EKF), BlackoutNavigator, Perception stubs, ROS2/Gazebo sim examples
- Dataflow: Sensors -> Fusion -> Nav -> Commands
- Safety: confidence gating, clearance checks, periodic reorientation
- Extensibility: plugin-like perception modules, ROS2 interfaces
