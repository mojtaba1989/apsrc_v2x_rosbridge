# APSRC V2X ROS Bridge Nodelet

This repository contains a ROS nodelet for bridging APSRC V2X (Vehicle-to-Everything) messages to ROS messages. The nodelet subscribes to UDP messages conforming to IEEE 1609.2 and J2735 standards and publishes corresponding ROS messages for Basic Safety Messages (BSMs), Signal Phase and Timing (SPaT), and Map Data.

## Dependencies
- ROS (Robot Operating System)
- ROS packages:
  - autoware_msgs
  - apsrc_msgs
  - network_interface
  - nodelet
  - roscpp
  - roslib
  - roslint
  - std_msgs
- C++11 or higher
- apsrc_v2x_viz for visualization (set False otherwise)  
(see https://github.com/mojtaba1989/apsrc_v2x_viz.git)

## Installation
1. Make sure you have ROS installed on your system.
2. Clone this repository into your ROS workspace.
3. Install asn libraries  
    1- Copy `lib/*.so` files to `/usr/lib/`  
    2- Run `sudo ldconfig`  
    3- Check with `ldconfig -p | grep <libname>` (without lib)  
4. Build the ROS workspace using `catkin_make`.

## Usage
```
roslaunch apsrc_v2x_rosbridge apsrc_v2x_rosbridge.launch ip:=<YOUR IP ADDR> port:=<PORT> node_name:=<"ANY"> visualization:=false
```   
- `IP` must correspond to an active IP on host machine.
- `node_name`: allows launching multiple instances of this node to use multiple radios simultaneously.
- `Port`: Choose different ports for different radios.
-  Confirm each radio's destination ip and port for successful netwrok configuration.

## Parameters
- `server_ip`: The IP address of the UDP server (default: `127.0.0.1`).
- `server_port`: The port of the UDP server (default: `1551`).

## Nodelet Functionality
- Subscribes to UDP messages conforming to IEEE 1609.2 and J2735 standards.
- Publishes ROS messages for Basic Safety Messages (BSMs), Signal Phase and Timing (SPaT), and Map Data.
- Handles UDP server initialization and message decoding.

This README is generated using https://chat.openai.com/.

