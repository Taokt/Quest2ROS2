import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, TransformStamped, Point
from quest2ros.msg import OVR2ROSInputs
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformListener, Buffer
import numpy as np
import tf_transformations
import time

# ROS 2 action client and gripper action message
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

# Deque for efficient fixed-length queues
from collections import deque

class BaseArmController(Node):
    def __init__(self, arm_name: str,robot_name:str , mirror: bool, base_frame_id:str , filter_window_size: int , end_effector_link_name: str , ctrl_prefix: str , gripper_action_topic: str):
        """
        Base class for controlling a robot arm using Quest 3 inputs.

        Args:
            arm_name (str): Arm name, e.g., 'left' or 'right', used for logging and distinction.
            mirror (bool): If True, maps the opposite hand controller to the arm 
                           (e.g., right hand controls left arm). Defaults to False.
        """
        #Initializing node
        node_name = f"{arm_name}_{robot_name}_arm_controller"
        super().__init__(node_name)
        self.get_logger().info(f"--- Initializing {node_name} ---")

        #Key parameter
        self.arm_name = arm_name
        self.robot_name = robot_name
        self.mirror = mirror
        self.base_frame_id = base_frame_id
        self.ctrl_prefix = ctrl_prefix
        self.gripper_action_topic = gripper_action_topic

        # Input param trans
        self.filter_window_size = filter_window_size
        self.end_effector_link_name = end_effector_link_name
        

        # Initialize parameter and interface
        self._init_parameters()
        self._init_variables()
        self._init_interfaces()
        
        self.commanded_trajectory_x = []
        self.commanded_trajectory_y = []
        self.commanded_trajectory_z = []
        self.executed_trajectory_x = []
        self.executed_trajectory_y = []
        self.executed_trajectory_z = []
        self.log_time = []

    def _init_parameters(self):
        #Moving-average filter configuration
        self.declare_parameter("filter_window_size", self.filter_window_size)
        self.filter_window_size = self.get_parameter("filter_window_size").value
        self.position_history = deque(maxlen=self.filter_window_size)
        self.orientation_history = deque(maxlen=self.filter_window_size)

    def _init_variables(self):
        #Initial basic parameter
        self.last_pose_stamped_always = None
        self.initial_orientation = None # Initial EEF orientation (from TF)
        self.initial_position = None    # Initial EEF position (from TF)
        self.first_received_quest_position = None # First Quest controller position
        self.first_received_quest_orientation = None # First Quest controller orientation
        self.current_robot_pose = None  # Current EEF position (from TF)
        self.allow_pose_update = True   # Enable/disable arm motion (toggled in inputs callback)
        self.is_gripper_closed = False  # Gripper state
        
    def _init_interfaces(self):
        #Initialize publisher,subscriber,TF,gripper
        ctrl_prefix = self.ctrl_prefix
        self.robot_target_pose_topic = f"{ctrl_prefix}/target_frame"

        #Mirror
        quest_side = ("right" if self.arm_name == "left" else "left") if self.mirror else self.arm_name
        
        self.quest_pose_topic = f"/q2r_{quest_side}_hand_pose"
        self.quest_inputs_topic = f"/q2r_{quest_side}_hand_inputs"
        self.get_logger().info(f"Ready. Listening to: {self.quest_pose_topic}")

        # TF2 for frame transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f"--- Robot {self.arm_name.upper()} Initialized ---")
        self.get_logger().info(f"Target Topic: {self.robot_target_pose_topic}")
        self.get_logger().info(f"Handler: {quest_side} (Mirror: {self.mirror})")

        #Subscriber
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.quest_pose_topic,
            self._pose_callback,
            10
        )
        self.input_subscription = self.create_subscription(
            OVR2ROSInputs,
            self.quest_inputs_topic,
            self._inputs_callback,
            10
        )

        #Publisher
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, self.robot_target_pose_topic, 10)

        # Gripper action client
        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_topic)
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Waiting for gripper action server: {self.gripper_action_topic}")

        self.get_logger().info(f'[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Ready to receive {self.quest_pose_topic} and publish to {self.robot_target_pose_topic}')



        
    def _get_robot_current_pose(self) -> (tuple[tuple[float, float, float], Quaternion]   | tuple[None, None]):
        """
        Query TF2 for the current end-effector (EEF) pose expressed in the base frame.

        Args:
            timeout_sec (float): Optional timeout for the TF lookup. Defaults to 0.0
                (non-blocking, return immediately with latest available transform).

        Returns:
            Tuple[Optional[Tuple[float, float, float]], Optional[Quaternion]]:
                - position: (x, y, z) in the base frame, or None on failure
                - orientation: Quaternion in the base frame, or None on failure
        """
        try:
            # Look up the latest available transform: base_frame <- end_effector_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.end_effector_link_name, rclpy.time.Time()
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_z = transform.transform.translation.z
            current_orientation = transform.transform.rotation
            self.current_robot_pose = (current_x, current_y, current_z)
            return (current_x, current_y, current_z), current_orientation
        except Exception as e:
            self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] can not get current robot transformation: {e}")
            return None, None
            
    def _apply_moving_average_filter(self, pose_stamped: PoseStamped):
        """
        Apply a moving-average filter to the incoming Quest pose.

        Returns:
            (avg_position, avg_orientation), where:
            - avg_position: np.ndarray shape (3,) or None while warming up
            - avg_orientation: np.ndarray shape (4,) [x, y, z, w] or None while warming up
        """

        # Append latest samples to the fixed-length deques
        self.position_history.append(
            np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z])
        )
        self.orientation_history.append(
            np.array([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, 
                      pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
        )

        # wait until the deques are full
        if len(self.position_history) < self.filter_window_size:
            self.get_logger().info(
                f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] "
                f"Filling filter... ({len(self.position_history)}/{self.filter_window_size})"
            )            
        
            return None, None 

        # Compute mean position over the window
        avg_position = np.mean(list(self.position_history), axis=0)

        # Compute mean orientation (simple linear average + renormalization)
        avg_orientation = np.mean(list(self.orientation_history), axis=0)
        norm = np.linalg.norm(avg_orientation)
        if norm < 1e-9:
            self.get_logger().warning(
                f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] "
                f"Average quaternion norm ~0; skipping this sample."
            )
            return None, None
        avg_orientation /= norm

        return avg_position, avg_orientation


    def _pose_callback(self, pose_stamped: PoseStamped):
        """
        Callback for Quest hand PoseStamped messages.
        Computes a robot target pose based on the robot's initial pose (anchor)
        and the hand's relative motion since that anchor.
        """
        # Always store the last received pose (for potential re-anchoring)
        self.last_pose_stamped_always = pose_stamped

        if not self.allow_pose_update:
            return

        # Query the current EEF pose from TF (in the base frame)
        robot_pos, robot_ori = self._get_robot_current_pose()
        if robot_pos is None or robot_ori is None:
            return 

        # Smooth the incoming Quest pose with the moving-average filter
        avg_quest_pos_np, avg_quest_ori_np = self._apply_moving_average_filter(pose_stamped)

        if avg_quest_pos_np is None:
            return

        # Set the initial orientation and position if not already set
        if self.initial_orientation is None:
            self.initial_orientation = robot_ori
            self.initial_position = robot_pos
            self.first_received_quest_position = avg_quest_pos_np # Use smoothed position
            self.first_received_quest_orientation = avg_quest_ori_np
            self.get_logger().info(
                f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] "
                f"Anchored robot pose: pos={self.initial_position}, ori={self.initial_orientation}"
            )
            return 

        offset_x = avg_quest_pos_np[0] - self.first_received_quest_position[0]
        offset_y = avg_quest_pos_np[1] - self.first_received_quest_position[1]
        offset_z = avg_quest_pos_np[2] - self.first_received_quest_position[2]

        q_quest_initial_np = self.first_received_quest_orientation
        q_quest_current_np = avg_quest_ori_np

        q_quest_initial_inv = tf_transformations.quaternion_inverse(q_quest_initial_np)
        q_quest_relative_rotation = tf_transformations.quaternion_multiply(q_quest_current_np, q_quest_initial_inv)

        q_robot_initial_np = np.array([self.initial_orientation.x, 
                                       self.initial_orientation.y,
                                       self.initial_orientation.z, 
                                       self.initial_orientation.w])
        
        q_target_robot_np = tf_transformations.quaternion_multiply(q_quest_relative_rotation, q_robot_initial_np)
        q_target_robot_np /= np.linalg.norm(q_target_robot_np)

        target_pose = Pose()
        target_pose.position.x = self.initial_position[0] + offset_x
        target_pose.position.y = self.initial_position[1] + offset_y
        target_pose.position.z = self.initial_position[2] + offset_z

        self.get_logger().info(f"x {target_pose.position.x:.4f} y {target_pose.position.y:.4f} z {target_pose.position.z:.4f}")
        target_pose.orientation.x = q_target_robot_np[0]
        target_pose.orientation.y = q_target_robot_np[1]
        target_pose.orientation.z = q_target_robot_np[2]
        target_pose.orientation.w = q_target_robot_np[3]


        # Visualization markers
        axes_marker = Marker()
        axes_marker.header.frame_id = self.base_frame_id
        axes_marker.header.stamp = self.get_clock().now().to_msg()
        axes_marker.ns = f"{self.arm_name}_{self.robot_name}_target_axes"
        axes_marker.id = 2
        axes_marker.type = Marker.LINE_LIST
        axes_marker.action = Marker.ADD
        axes_marker.pose.position = target_pose.position
        axes_marker.pose.orientation = target_pose.orientation
        axes_marker.scale.x = 0.02 

        axis_length = 0.2
        axes_marker.points = [
            Point(x=0.0,y=0.0,z=0.0), Point(x=axis_length,y=0.0,z=0.0),  # X
            Point(x=0.0,y=0.0,z=0.0), Point(x=0.0,y=axis_length,z=0.0),  # Y
            Point(x=0.0,y=0.0,z=0.0), Point(x=0.0,y=0.0,z=axis_length)   # Z
        ]
        axes_marker.colors = [
            ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0), ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0),
            ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0), ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0),
            ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0), ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0)
        ]
        self.marker_pub.publish(axes_marker)

        # Publish the target pose 
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = self.base_frame_id
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.pose = target_pose
        self.target_pose_publisher.publish(target_pose_stamped)
        self.last_executed_target_pose = target_pose
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Published new target pose.")
    
    def _toggle_gripper(self):
        """
        Toggle the gripper: send an action goal to open or close it depending on
        the current state. Updates internal state after the action completes.
        """
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Gripper action server not available.")
            return

        goal_msg = GripperCommand.Goal()

        if self.is_gripper_closed:
            goal_msg.command.position = 0.0 # 0.0 Fully open
            goal_msg.command.max_effort = 5.0 
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Sending goal to open gripper...")
        else:
            goal_msg.command.position = 0.05 # 0.05 Fully closed
            goal_msg.command.max_effort = 5.0 
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Sending goal to close gripper...")

        # Send goal and update state
        self.gripper_action_client.send_goal_async(goal_msg)
        self.is_gripper_closed = not self.is_gripper_closed

    def _inputs_callback(self, msg: OVR2ROSInputs):
        """
        Callback for Quest 3 controller inputs.

        - Upper button (button_upper): toggle gripper open/close.
        - Lower button (button_lower): toggle pose streaming on/off and recenter anchors.
        """
        # Ensure edge-detection flags exist (one action per press)
        if not hasattr(self, '_button_upper_pressed_state'):
            self._button_upper_pressed_state = False
        if not hasattr(self, '_button_lower_pressed_state'):
            self._button_lower_pressed_state = False

        # Upper button: Toggle gripper
        if msg.button_upper and not self._button_upper_pressed_state:
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Upper button pressed. Toggling gripper...")
            self._toggle_gripper()
            
        # Update upper button press state
        self._button_upper_pressed_state = msg.button_upper

        # Lower button: Toggle pose updates and re-anchor
        if msg.button_lower and not self._button_lower_pressed_state:
            # Flip the pose-streaming gate
            self.allow_pose_update = not self.allow_pose_update
            
            state_msg = "ENABLED (publishing poses)" if self.allow_pose_update else "DISABLED (hold position)"
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Lower button pressed: pose streaming is now **{state_msg}**。")
            
            robot_pos, robot_ori = self._get_robot_current_pose()
            
            if self.last_pose_stamped_always is not None:
                pass

            if robot_pos is not None and robot_ori is not None:
                # Set robot anchor to the current EEF pose
                self.initial_position = robot_pos
                self.initial_orientation = robot_ori
                self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_name.upper()} Arm] Robot anchor reset to current pose. ")
                
                # Force _pose_callback to re-anchor Quest on the next smoothed sample
                self.initial_orientation = None 
                self.first_received_quest_position = None
                self.first_received_quest_orientation = None

            # Clear filter history
            self.position_history.clear()
            self.orientation_history.clear()

        # Update lower button press state
        self._button_lower_pressed_state = msg.button_lower