# Quest2ros2
A framework for using Meta Quest 2/3 VR controllers `quest2ros` to teleoperate a robot system through ROS 2 and ROS–TCP communication. 

## System Requirements
### Operating Systems
- Ubuntu 22.04 LTS
### ROS Version
- ROS 2 Humble
### VR Hardware
- Meta Quest 2 / Quest 3 headset with hand controllers
### Prerequisites
- The robot must have a ROS 2 controller that supports **Cartesian end-effector control**.


## Setup and Configuration

1. Install ROS 2 Humble on Ubuntu.

`https://docs.ros.org/en/humble/Installation.html`

2. Clone and configure [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

`git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git`

and switch to `main-ros2` branch.

**NOTE:** As of September 2025, the official ROS–TCP–Endpoint repository is not fully compatible with our current implementation due to several unresolved issues. Please apply the following manual changes to ensure compatibility.


a) In the `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py`, replace:

`message_json = data.decode("utf-8")[:-1]`

with:

`message_json = data.decode("utf-8")`

b) In both `ROS-TCP-Endpoint/ros_tcp_endpoint/server.py` and `ROS-TCP-Endpoint/launch/endpoint.py`, update the `ROS_IP` variable from `"0.0.0.0"` to match your device's actual IP address.

c) Copy the `ros_msg_converter.py` and `publisher.py` from `Files_for_ros_tcp` folder in the `quest2ros2` repository, and replace the files with the same names in the `ROS-TCP-Endpoint` repository.

d) Rebuild `ros-tcp-endpoint` package to apply the changes:

```
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

3. Install and configure `quest2ros` app on your VR headset.

Follow instructions at: `https://quest2ros.github.io/`

4. Create a new ROS 2 package for the message definitions:

`ros2 pkg create quest2ros --build-type ament_python`

This command generates an empty package named `quest2ros`.

Navigate into this package and create a folder named `msg`:

```
cd quest2ros
mkdir msg
```

Then copy the `.msg` files from `quest2ros2/msg` into the newly created `quest2ros/msg` folder.

5. Clone this project

User should update the Base Frame ID (`base_link`), End-Effector Link Name ((`left_arm_link_ee`, `right_arm_link_ee`)) , and ROS 2 Topic names within the `_configure_robot_params` in `robot_arm_controller_base.py` method to match your robot's setup. 

Ensure you also customize the namespace (e.g., replacing bh_robot) in the controller topics to correctly address your specific robot's action servers.

6. Build your ROS 2 Humble workspace:

```
colcon build
source install/setup.bash
```

7. Launch ROS–TCP Endpoint  

`ros2 launch ros_tcp_endpoint endpoint.py`

8. Configure the connection in the Quest2ROS app.

On your VR headset, open the Quest2ROS app, set your device’s IP address (<YOUR_IP>) and the same port number as defined in endpoint.py, then press Apply to confirm.

9. Run CheckTCPconnection

To verify VR → ROS communication:

`ros2 run q2r_bringup CheckTCPconnection`

This subscribes to `/q2r_right_hand_pose` and shows the VR controller position/orientation.

This node is used to confirm whether the right-hand controller is connected properly. To test the left-hand controller instead, change the subscribed topic from `/q2r_right_hand_pose` to `/q2r_left_hand_pose`.

10. Run left_arm_controller.py and right_arm_controller.py

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


## Debugging and Simulation
### VR Input Simulator:SimulationInpt.py

The file SimulationInpt.py is a VR Input Simulator. It publishes fixed input and velocity data (OVR2ROSInputs and Twist) at 1 Hz for testing purposes, allowing you to debug your control logic without the VR headset connected.

### How to run

`ros2 run q2r_bringup SimulationInpt`
