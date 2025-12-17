# Quest2ROS2
A framework for using Meta Quest 2/3 VR controllers to teleoperate a robot system through ROS2 and ROS–TCP communication. Built based on [Quest2ROS](https://quest2ros.github.io/).\
*Accepted to **HRI 2026***

https://github.com/user-attachments/assets/f153b410-1828-4f67-8ec8-fc7ae9254131

## System Requirements
### Operating Systems
- Ubuntu 22.04 LTS
### ROS Version
- ROS2 Humble
### VR Hardware
- Meta Quest 2 / Quest 3 headset with hand controllers
### Prerequisites
- The robot must have a ROS2 controller that supports **Cartesian end-effector control**.


## Setup and Configuration

1. For the VR device, install and configure the `quest2ros` app on your VR headset. Follow instructions at: [Quest2ROS](https://quest2ros.github.io/)

2. On PC side, install ROS2 Humble on Ubuntu: `https://docs.ros.org/en/humble/Installation.html`, make sure to install Python modules numpy and tf_transformation.

3. Assume the workspace path is `workspace`, go to `workspace/src`, Clone and configure [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
`git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git` and do `cd ROS-TCP-Endpoint` follow with `git switch main-ros2` to switch to the `main-ros2` branch.

4. Under `workspace/src`, clone this project with `git clone https://github.com/Taokt/Quest2ROS2.git`

5. **NOTE:** As of September 2025, the official ROS–TCP–Endpoint repository is not fully compatible with our current implementation due to several unresolved issues. Please apply the following manual changes to ensure compatibility.

    - In line 125 of the file `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py`, replace: `message_json = data.decode("utf-8")[:-1]` with `message_json = data.decode("utf-8")`

    - In both `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py` and `ROS-TCP-Endpoint/launch/endpoint.py`, update the `ROS_IP` variable from the default `"0.0.0.0"` to match your robot's actual IP address.

    - Under path `/src`, do `cp Quest2ROS2/Files_for_ros_tcp/* ROS-TCP-Endpoint/ros_tcp_endpoint` to replace the files with the same names.

    - Rebuild `ros-tcp-endpoint` package to apply the changes:
```
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

Unlike ROS1, ROS2 perfer custom message to be in a sepreate package (https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html), so the next step is to create the custom messages for our Quest2ROS2 package.

6.  Under `/src`, create a new ROS 2 package for the custom messages with:

```
ros2 pkg create --build-type ament_cmake quest2ros2_msg
```

This command generates an empty package named `quest2ros2_msg`.
Then copy the corrsponding required files into the newly created package:

Under `/src`, do
```
cp -r Quest2ROS2/Files_for_msg_pkg/* quest2ros2_msg/
```

7. Configure this project: User should update the Base Frame ID (`base_link`), End-Effector Link Name ((`left_arm_link_ee`, `right_arm_link_ee`)) , and ROS 2 Topic names within the `_configure_robot_params` in `robot_arm_controller_base.py` method to match your robot's setup. 

Ensure you also customize the namespace (e.g., replacing bh_robot) in the controller topics to correctly address your specific robot's action servers.

8. Build your ROS2 Humble workspace:

```
colcon build
source install/setup.bash
```

9. Quick varify: One way to varify the success building of `Quest2ROS2` package is to try run VR input simulator inclueded in file `simulationInput.py`. It publishes some simulated input and velocity data (OVR2ROSInputs and Twist) at 1 Hz for testing purposes, allowing you to debug the package build status. Try it with:
```
ros2 run q2r_bringup simulationInput
```
If it successfully publishes data, one is safe to move to the nex step, which is to build the connection to the robot.

10. Launch ROS–TCP Endpoint  

```
ros2 launch ros_tcp_endpoint endpoint.py
```

11. Configure the connection in the Quest2ROS app on the VR device.

On your VR headset, open the Quest2ROS app, set your robot’s IP address (<YOUR_IP>) and the same port number as defined in `endpoint.py`, then press Apply to confirm.

12. Run CheckTCPconnection

To verify VR → ROS communication:

`ros2 run q2r_bringup CheckTCPconnection`

This subscribes to `/q2r_right_hand_pose` and shows the VR controller position/orientation.

This node is used to confirm whether the right-hand controller is connected properly. To test the left-hand controller instead, change the subscribed topic from `/q2r_right_hand_pose` to `/q2r_left_hand_pose`.

13. Run left_arm_controller.py and right_arm_controller.py

Running the code, move the handle to control the movement of the robotic arm

```
ros2 run q2r_bringup left_arm_controller
ros2 run q2r_bringup right_arm_controller
```

## Running and Interaction

### Start your robot hardware, RViz, and controller.

Ensure your physical robot, RViz visualization, and motion controller are all running.

### Expected Output

- RViz visualization of dual-arm robot.

- Real-time motion following VR hand controllers.

### Button functions

- Upper Button : Toggles gripper state (each press switchs between open and close).

- Lower Button : Toggles the robot arm movement (each press switches between enable and disable), and also pauses/resumes pose streaming while performing an Anchor Reset that snaps the virtual target back to the robot’s current position to prevent movement jumps due to kinematic limits or drift.

### Change to "Mirror" Mode
- Change the control mode to "Mirror" by set `self.mirror` to `True` in the `__init__` method of `BaseArmController` class inside `robot_arm_controller_base.py`. For more information please refer to the report.

