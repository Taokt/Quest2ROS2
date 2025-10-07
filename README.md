# Quest2ros2
A framework for using Meta Quest 2/3 VR controllers `quest2ros` to remotely control a robot system through ROS 2 and ROS–TCP communication.


## System Requirements
### Operating Systems
- Ubuntu 22.04 LTS
### ROS Version
- ROS 2 Humble
### VR Hardware
- Meta Quest 2 / Quest 3 headset with hand controllers


## Installation Guide

1. Install ROS 2 Humble on Ubuntu.

`https://docs.ros.org/en/humble/Installation.html`

2. Clone and configure [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

`https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git`

and switch to `main-ros2` branch.

**NOTE:** As of September 2025, the official ROS–TCP–Endpoint repository is not fully compatible with our current implementation due to several unresolved issues. Please apply the following manual changes to ensure compatibility.


a) In the `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py`, replace:

`message_json = data.decode("utf-8")[:-1]`

with:

`message_json = data.decode("utf-8")`

b) In both `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py` and `ROS-TCP-Endpoint/ros_tcp_endpoint/endpoint.py`, update the `ROS_IP` variable to match your device's actual IP address.

c) Copy the `ros_msg_converter.py` and `publisher.py` from `Files_for_ros_tcp` folder in the `quest2ros2` repository, and and replace the files with the same names in the `ROS-TCP-Endpoint` repository.


3. Install and configure `quest2ros` app on your VR headset.

Follow instructions at: `https://quest2ros.github.io/`

4. Create a new ROS 2 package for the message definitions:

`ros2 pkg create quest2ros --build-type ament_python`

Copy the `.msg` files from `msg`  into `msg/` and rebuild.

5. Build your ROS 2 Humble workspace:

`colcon build`

6. Launch ROS–TCP Endpoint  

`ros2 launch ros_tcp_endpoint endpoint.py`

7. Set your device’s IP address (<YOUR_IP>) and the same port number in the Quest2ros app of VR headset, then press Apply.

8. Run ros2quest demo

`ros2 run q2r_bringup ros2quest.py`

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

- Lower-Button presses (A = right, X = left) toggle the robot arm movement. (Press once to enable, press again to disable.)


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
