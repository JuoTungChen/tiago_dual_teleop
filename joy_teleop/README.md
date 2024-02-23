joy_teleop
==========

A configurable node to map joystick controls to robot teleoperation commands

## Setup Guide

1. Connect Joystick to computer
2. In one terminal, source the path and run the following to launch gazebo simulation for Tiago++ with Omni-base and Pal-Gripper
    ```
    roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true base_type:=omni_base end_effector_left:=pal-gripper end_effector_right:=pal-gripper
    ```
3. In another terminal, source the path and run:
    ```
    roslaunch joy_teleop teleop.launch
    ```