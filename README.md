# Control Algorithm for Parachute Guidance System

This project focuses on the design and simulation of a control algorithm
for an autonomous ram-air parachute guidance system using L1 navigation.
The work follows a simulation-first approach prior to embedded and
hardware testing.

## System Overview
- Ram-air parachute steered using control lines
- Control lines actuated via servo motors
- Servo commands generated from guidance error relative to waypoints

## Guidance Algorithm
- L1 navigation guidance used for waypoint tracking
- Cross-track error and heading correction computed from waypoint geometry
- Algorithm designed to guide the parachute toward a predefined target region

## MATLAB Simulation
The `matlab_simulation/` directory contains the mathematical models and
trajectory simulations used to analyze and validate guidance behavior.

- Two variants of the L1 guidance implementation are explored
- Waypoint tracking plots generated for trajectory analysis
- Comparative evaluation used to select a suitable guidance approach

## Simulation Results
The simulation results demonstrate stable waypoint tracking behavior
and provide insight into trajectory response prior to hardware deployment.

(Refer to the plots directory for waypoint tracking visualizations.)

## Embedded Implementation 
An ESP32-based embedded control system is planned to implement the
validated guidance logic using servo-actuated control lines.

**Status:**  
The embedded implementation is currently under development and will
be refined to closely match the validated MATLAB model before testing.

## Project Status
- MATLAB-based simulation complete
- Embedded control implementation pending
- Hardware-in-loop and field testing planned

