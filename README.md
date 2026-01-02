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

3. Assume the workspace path is `workspace`, go to `workspace/src`, clone this project with `git clone https://github.com/Taokt/Quest2ROS2.git`

4. **NOTE:** The upstream `ROS–TCP–Endpoint` repository is not fully compatible with our current implementation. To avoid manual patching and reproduction issues, please use our maintained package `ros_tcp_communication` instead.

    Under `workspace/src`, install & build:

    `git clone https://github.com/guguroro/ros_tcp_communication.git`

    `cd ..`

    `colcon build --packages-select ros_tcp_communication`

    `source install/setup.bash`

    In `ros_tcp_communication/launch/endpoint.py`, update the ROS_IP variable in the launch file from the default "0.0.0.0" to match your robot's actual IP address.

5.  The `Quest2ROS` application only looking for ros2 message with the specific package name `quest2ros`, therefore one would have to create a seperate package for the the custom messages definitions. 
    Under `/src`, create a new ROS2 package as

    ```
    ros2 pkg create --build-type ament_cmake quest2ros
    ```

    This command generates an empty package named `quest2ros`.
    Then copy the corrsponding required files into the newly created package:

    Under `/src`, do
    ```
    cp -r Quest2ROS2/Files_for_msg_pkg/* quest2ros/
    ```

6. Configure this project: 

    All robot-specific parameters are explicitly exposed in `left_arm_controller.py` and `right_arm_controller.py`. Users should modify the constructor arguments in these two files to match their robot setup.

    The key parameters to customize include:
    - `robot_name`: your robot type or model name (e.g. kuka)
    - `arm_name`: arm identifier (e.g. left or right)
    - `base_frame_id`: root reference frame of the robot (e.g. bh_robot_base)
    - `end_effector_link_name`: end-effector link name for each arm
    - `ctrl_prefix`: namespace of the arm inverse kinematics controller
    - `gripper_action_topic`: action server topic for gripper control
    - `filter_window_size`: moving average filter size for motion smoothing
    - `mirror`: whether to mirror controller input between two arms, note this flag has to be the same for both `left_arm_controller.py` and `right_arm_controller.py`.

7. Build your ROS2 Humble workspace:

    ```
    colcon build
    source install/setup.bash
    ```

8. Quick varify: One way to varify the success building of `Quest2ROS2` package is to try run VR input simulator inclueded in file `SimulationInput.py`. It publishes simulated Quest controller data at 30 Hz, including button states, hand pose and velocity. Try it with:

    ```
    ros2 run q2r2_bringup SimulationInput
    ```

    Optional modes:
    ```
    # Simulate LEFT Hand:
    ros2 run q2r2_bringup SimulationInput --ros-args -p side:=left

    # Simulate Only Velocity (Twist) Mode:
    ros2 run q2r2_bringup SimulationInput --ros-args -p mode:=velocity

    # Simulate Left Hand with Only Teleop Data (Pose + Inputs):
    ros2 run q2r2_bringup SimulationInput --ros-args -p side:=left -p mode:=teleop
    ```

    If you can see the topics publishing (e.g., via `ros2 topic list` or `ros2 topic echo`), you can proceed to the next step.

9. Launch ROS–TCP Endpoint  

    ```
    ros2 launch ros_tcp_endpoint endpoint.py
    ```

10. Configure the connection in the Quest2ROS app on the VR device.

    On your VR headset, open the Quest2ROS app, set your robot’s IP address (<YOUR_IP>) and the same port number as defined in `endpoint.py`, then press Apply to confirm.

11. Run CheckTCPconnection

    To verify VR → ROS2 communication:

    `ros2 run q2r2_bringup CheckTCPconnection`

    This node subscribes to all Quest topics for both hands and all message types:
    - `/q2r_left_hand_pose`
    - `/q2r_left_hand_inputs` 
    - `/q2r_left_hand_twist` 
    - `/q2r_right_hand_pose` 
    - `/q2r_right_hand_inputs` 
    - `/q2r_right_hand_twist` 

    If the connection is working, it prints the latest received values every second (pose / inputs / twist).

    If no messages are received more than 2 seconds, it reports a timeout.

12. Run left_arm_controller.py and right_arm_controller.py

    Running the code, move the handle to control the movement of the robotic arm

    ```
    ros2 run q2r2_bringup left_arm_controller
    ros2 run q2r2_bringup right_arm_controller
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


