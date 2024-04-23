# vive teleop for tiago++ with omni base

This ros package provides launch files to map HTC vive VR cobtrollers to control the Tiago++ robot with an Omni base. 

## Features

- Maps Vive controller inputs to robot commands
- Maps gamepad controller inputs to robot commands
- Supports teleoperation of of the whole body (base, torso, head, both arms)

## Installation

Clone this repository into your catkin workspace, then use `catkin build` to build:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/vive_teleop.git
cd ..
catkin build
```

## Usage
### Launch Files
There are two launch files for robot teleop:
1. ```vive_teleop.launch``` is for controlling the robot using the vive controllers
2. ```gamepad_teleop.launch``` is for controlling the robot using any type of gamepad (joystick) interfaces

### Gamepad interface mapping

The `gamepad_teleop.launch` file is designed to allow control of the robot using a gamepad or joystick interface. The specific button-to-command mappings depend on the type of gamepad you are using, but a general mapping might look like this:

- **Button A**: Activates the mobile base control mode
- **Button B**: Activates the right arm control mode
- **Button X**: Activates the left arm control mode
- **Button Y**: Activates the head control mode

When in mobile base control mode:
- **Left Stick (Vertical)**: Controls the forward and backward movement of the mobile base.
- **Left Stick (Horizontal)**: Controls the left and right strafing movement of the mobile base.
- **Right Stick (Horizontal)**: Controls the rotation of the mobile base.
- **D-pad (Vertical)**: Controls the height of the torso.

When in arm (left or right) control mode
- **Left Stick (Vertical)**: Controls the x translation of the end-effector.
- **Left Stick (Horizontal)**: Controls the y translation of the end-effector.
- **Right Stick (Vertical)**: Controls the z translation of the end-effector.
- **Right Stick (Horizontal)**: Controls the yaw rotation of the end-effector.
- **D-pad (Vertical)**: Controls the pitch rotation of the end-effector.
- **D-pad (Horizontal)**: Controls the roll rotation of the end-effector.

When in head control mode:
- **Left Stick (Vertical)**: Controls the pitch rotation of the head.
- **Left Stick (Horizontal)**: Controls the yaw rotation of the head.

In any mode:
- **Right Trigger**: Controls the openning and closing of the right gripper.
- **Left Trigger**: Controls the openning and closing of the left gripper.

### Launch File Arguments

The `vive_teleop.launch` file accepts several arguments that allow you to customize its behavior:

- `sim` (default: `true`): If set to `true`, the node operates in simulation mode. If set to `false`, the node operates with a real robot.

- `record` (default: `false`): If set to `true`, the node records data during operation. This can be useful for debugging or analysis.

- `rviz` (default: `true`): If set to `true`, the node launches RViz for visualizing the robot and its sensor data.

You can set these arguments when launching the node. For example, to launch the node in non-simulation mode without RViz, you can run:

```bash
roslaunch vive_teleop vive_teleop.launch sim:=false rviz:=false
```