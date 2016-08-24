# GOW - Hackathon Globo 2016

This repository contains the code implemented at the Hackathon Globo 2016, Rio de Janeiro, Brazil.

The project GOW is a Teleoperation system between a Human (wearing a VR gear in front of a Kinect sensor) and a Humanoid Robot ([NAO](https://www.ald.softbankrobotics.com/en/cool-robots/nao)). Thus, the robot will repeat the arms and neck movement done by a person placed in front of a Kinect. Also, the image seen by the robot will be represented in the VR gear weared by the person. This is close to a virtual teleportation of a human to a robot body. The idea of this project is to inovate the way people interact with television programs.

## Requirements
This project requires the following items:

1.  Nao Robot

2.  VR Gear

3.  Kinect Sensor

## Code
This work was implemented mostly in Python and C++ in the ROS enviroment using Ubuntu 14.04.

=======
## Launch Files

*** markers_only.launch ***
Use it if you mean to get data directly from the kinect connected.
Else, you have to use rosbag to publish information of skeleton.

THEN choose one of the next launch files

*** teleoperation_DQ.launch ***
Use this one
Runs using inverse kinematics and dual quaternions

*** kinect_only.launch ***
Runs the kinect_no_nao_publisher.py - DO NOT USE	

*** plot_teleoperation_DQ ***
DO NOT USE.

https://github.com/pirobot/skeleton_markers

https://github.com/ros-drivers/openni_tracker.git 

Baixar essas versoes do driver. Foi o que deu certo pra mim.
https://code.google.com/p/simple-openni/downloads/detail?name=OpenNI_NITE_Installer-Linux64-0.27.zip&can=3&q=



