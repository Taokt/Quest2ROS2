# Quest2ros2
A framework for using Meta Quest 2/3 VR controllers `quest2ros` to remotely control a KUKA dual-arm robot through ROS 2 and ROS–TCP communication.


## System Requirements
### Operating Systems
- Ubuntu 22.04 LTS
### ROS Version
- ROS 2 Humble
### VR Dependencies
- Meta Quest 2 / Quest 3 headset with hand controllers
### Robot platform
- KUKA dual-arm robot

## Installation Guide

1. Install ROS 2 Humble on Ubuntu.

`https://docs.ros.org/en/humble/Installation.html`

2. Clone and configure [ROS-TCP-Endpoint](https://github.com/guguroro/ros_tcp_communication)

`git clone git@github.com:guguroro/ros_tcp_communication.git`

**NOTE:** ROS-TCP-Endpoint has some errors. We have submitted a pull request but have not received any feedback yet. Therefore, users are required to modify it themselves when using it.

**Key Modifications:** The main adjustments were made in three places:

a) In `server.py`, the decoding line was updated to **`message_json = data.decode("utf-8")`** to correctly handle incoming data encoding.

b) Place the `ros_msg_converter.py` and `publisher.py` into the root directory of the cloned ROS-TCP-Endpoint repository

c) In `server.py`,Update the ROS_IP variable to your device's actual IP address


3. Install and configure `quest2ros` on your VR headset.

Follow instructions at: `https://quest2ros.github.io/`

4. Create a new ROS 2 package for the message definitions:

`ros2 pkg create quest2ros --build-type ament_python`

Copy the `.msg` files into `msg/` and rebuild.


## Demo

### Launch Robot Simulation

```
ros2 launch lbr_bring dual_arm_mock.launch.py
ros2 launch lbr_bring dual_arm_move_group.launch.py rviz:=True
```

### Launch ROS–TCP Endpoint

`ros2 launch ros_tcp_endpoint endpoint.py`

### Start Quest2ROS App

On the Meta Quest headset, start the `quest2ros` app and set the correct IP.

### Run Control Node

Control the left or right arm via:

```
ros2 run q2r_bringup left_arm_controller.py
ros2 run q2r_bringup right_arm_controller.py
```

### Expected Output

- RViz visualization of dual-arm robot.

- Real-time motion following VR hand controllers.

- Lower-Button presses (A = right, X = left) temporarily disable robot arm movement until released.

- Demo video coming soon.

## Instructions for Use

### Check TCP Connection

To verify VR → ROS communication:

`ros2 run q2r_bringup CheckTCPconnection.py`

This subscribes to `/q2r_right_hand_pose` and shows the VR controller position/orientation.

### Button functions

- Upper Button: Toggles Gripper state (Open/Close)

- Lower Button: Pauses/Resumes pose streaming and performs an Anchor Reset. This immediately snaps the virtual target back to the robot's current position, eliminating movement jumps caused by kinematic limits or drift.

### Customization

- Update `group_name`, `link_name`, and `frame_id` to match your robot setup.

- Update `namespace` (e.g. replace `bh_robot` with your namespace).



### Notes

- Create a separate package for message types, otherwise `quest2ros2` cannot find them.
- If the robot arm jitters during control, recalibrate the robot’s sensors.

## Debugging and Simulation
### VR Input Simulator:ros2quest.py

The file ros2quest.py is a VR Input Simulator. It publishes fixed input and velocity data (OVR2ROSInputs and Twist) at 1 Hz for testing purposes, allowing you to debug your control logic without the VR headset connected.

### How to run

`ros2 run q2r_bringup ros2quest.py`
