# Quest2ros2
Diescription:
Using vr-handler to remote control kuka dual-arm robot.


# Pre-condition:
ROS2: Humble
ROS-TCP-Communition:`git@github.com:guguroro/ros_tcp_communication.git`,change the IP to your IP.

You need to install `quest2ros` in your VR header:`https://quest2ros.github.io/ `

You need a meta quest 2/3

You need to create a seperate pkg to save the message type:`OVR2ROSHapticFeedback.msg` &&  `OVR2ROSInputs.msg`

    e.g. create a pkg named quest2ros,then you can find the message type by : `from quest2ros.msg import OVR2ROSInputs`

# Launch the program

First,Launch your robot, using command like:

`ros2 launch lbr_bring dual_arm_mock.launch.py` 

and 

`ros2 launch lbr_bring dual_arm_move_group.launch.py rviz:=True` 

Then,Launch`ros_tcp_endpoint`to build communication between VR and robot.

start your `quest2ros` App in your VR,and set the correct IP.

Then,launch the ros_tcp_endpoint,using

`ros2 launch ros_tcp_endpoint endpoint.py`

Then,control robot arm using the vr-handler.Using command:

`ros2 run q2r_bringup Q2R_control_goal_right.py`

or

`ros2 run q2r_bringup Q2R_control_goal_left.py`

    

    
# Usage

If you want to build connection directly,use: 

`ros2 run py01_topic CheckTCPconnection.py`

this will subscribe `/q2r_right_hand_pose`topic,show you the right vr handler's position and orientation.you can change it to any topic you want.

`Q2R_control_goal_left.py` is used to control left arm
`Q2R_control_goal_right.py` is used to control right arm

Please change the `group_name`,`link_name`,`frame_id` to your real setting.

Please change the server's namespace to your real namespace.
    e.g. change `bh_robot` to your namespace or delete it

For the `button_lower`,means button `A` for right handler and button `X` for the left handler.while you presssing them,the robot arm will not be controlled until you release them. 

# Important Note
Create a seperate package for message type and copy the msg out to the pckage, otherwsie it can not be found be quest2ros2 

If the robot arm always jumping when you trying to control it,please recalibrate the sensor of robot arm.

