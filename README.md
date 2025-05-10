# Team Underwater Perception for Event Response (TUPER)

## Introduction

This repository contains the control and filter code for the TUPER project, which aims to control a fleet of underwater vehicles based on acoustic ranging information provided by two surface vehicles. It is designed to be used with ROS2 and is compatible with the SMARC Unity Simulation, as well as real-life vehicles. The control scheme outputs throttle and steering commands in the general `std_msgs/Float32` format. Hence, it can be repurposed for any vehicle that can accept these commands.

Overview of the repository:
- `BehaviorTree.CPP` and `BehaviorTree.ROS2`: C++ library for behavior trees, which are used to design the control logic of the vehicles. These are forked from the original repository and modified to suit our needs.
- `docker`: Dockerfile and scripts for building and running the project in a containerized environment.
- `formation_controller`: most code related to the formation control of the vehicles. It does not contain behavior trees, but rather utility nodes and action servers needed for the behavior trees to work.
- `position_filter`: UKF code.
- `tuper_btcpp`: custom tree nodes and behavior trees. If you need to write new nodes, here is where to add. Most likely you will only need to modify trees with Groot2.
- `tuper_inferfaces`: custom messages, services and actions for the project.

SAM/Unity specific packages: 
- `sam_thruster_relay`: relay `std_msgs/Float32` to `smarc_msgs/ThrusterRPM` and `sam_msgs/ThrusterAngles` to be compatible with the Unity simulation.
- `tuper_sim_utils`: contains `ping_synchronizer` to coordinate the ping frequency of the two leaders, and `string_stamped_processing` to process the acoustic message data in Unity.

Real-life specific packages: 
- `arduagent`: code for controlling Ardupilot-based vehicles (ArduRover, ArduSub, etc..). It also contains MQTT nodes to send and receive data between ArduPilot and MQTT broker.
- `monitoring`: code to be run on ground station (eg. your own laptop) to monitor the vehicles via data received from MQTT.

## Tutorials

### - [Tutorial 0: Installation](docs/installation.md)
### - [Tutorial 1: Running the code](docs/running_the_code.md)