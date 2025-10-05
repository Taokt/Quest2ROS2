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
    def __init__(self, arm_name: str, robot_type: str):
        """
        Base class for controlling a robot arm using Quest 3 inputs.

        Args:
            arm_name (str): Arm name, e.g., 'left' or 'right', used for logging and distinction.
            robot_type (str): Robot type, e.g., 'kuka', 'franka', or 'z1'.
        """
        
        node_name_str = f"{arm_name}_{robot_type}_arm_controller"
        super().__init__(node_name_str)
        self._node_name = node_name_str 

        self.arm_name = arm_name
        self.robot_type = robot_type

        # Configure robot-specific parameters based on type and arm side
        self._configure_robot_params()


        self.last_pose_stamped_always = None
        self.initial_orientation = None # Initial EEF orientation (from TF)
        self.initial_position = None    # Initial EEF position (from TF)
        self.first_received_quest_position = None # First Quest controller position
        self.first_received_quest_orientation = None # First Quest controller orientation
        self.current_robot_pose = None  # Current EEF position (from TF)
        self.allow_pose_update = True   # Enable/disable arm motion (toggled in inputs callback)

        # Gripper state
        self.is_gripper_closed = False
        
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

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, self.robot_target_pose_topic, 10)

        # TF2 for frame transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Moving-average filter configuration
        self.filter_window_size = 20 # Window length
        self.position_history = deque(maxlen=self.filter_window_size)
        self.orientation_history = deque(maxlen=self.filter_window_size)

        # Gripper action client
        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_topic)
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Waiting for gripper action server: {self.gripper_action_topic}")

        self.get_logger().info(f'[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Ready to receive {self.quest_pose_topic} and publish to {self.robot_target_pose_topic}')
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Node name: {self._node_name}")

        self.commanded_trajectory_x = []
        self.commanded_trajectory_y = []
        self.commanded_trajectory_z = []
        self.executed_trajectory_x = []
        self.executed_trajectory_y = []
        self.executed_trajectory_z = []
        self.log_time = []


    def _configure_robot_params(self):
        """
        Configure robot-specific parameters (topics and TF frames) based on the
        robot type and arm side.
        """
        topic_name_suffix = "/target_frame" 

        if self.robot_type == "franka":
            self.base_frame_id = "base"
            self.end_effector_link_name = "fr3_hand_tcp"
            controller_prefix = "/cartesian_impedance_controller"
            self.quest_pose_topic = f"/q2r_{self.arm_name}_hand_pose" 
            self.quest_inputs_topic = f"/q2r_{self.arm_name}_hand_inputs"
            self.robot_target_pose_topic = controller_prefix + topic_name_suffix
            self.gripper_action_topic = f"/franka_gripper/gripper_action" 

        elif self.robot_type == "kuka":
            self.base_frame_id = "bh_robot_base" 

            if self.arm_name == "left":
                self.end_effector_link_name = "left_arm_link_ee"
                controller_prefix = "/bh_robot/left_arm_clik_controller"
                self.quest_pose_topic = "/q2r_left_hand_pose"
                self.quest_inputs_topic = "/q2r_left_hand_inputs"
                self.gripper_action_topic = "/bh_robot/left_arm_gripper_action_controller/gripper_cmd"
            elif self.arm_name == "right":
                self.end_effector_link_name = "right_arm_link_ee" 
                controller_prefix = "/bh_robot/right_arm_clik_controller" 
                self.quest_pose_topic = "/q2r_right_hand_pose"
                self.quest_inputs_topic = "/q2r_right_hand_inputs"
                self.gripper_action_topic = "/bh_robot/right_arm_gripper_action_controller/gripper_cmd"
            else:
                self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] KUKA 机器人未知的手臂名称: {self.arm_name}。期望为 'left' 或 'right'。")
                raise ValueError(f"Unknown KUKA arm name: {self.arm_name}. Expected 'left' or 'right'. ")

            # Final command topic for the chosen arm's controller
            self.robot_target_pose_topic = controller_prefix + topic_name_suffix

        elif self.robot_type == "z1":
            # TF frames
            self.base_frame_id = "link00"
            self.end_effector_link_name = "link06"
            # Controller namespace
            controller_prefix = "/cartesian_impedance_controller"
            # Quest input topics (built from arm side; adjust if Z1 uses a single-arm setup)
            self.quest_pose_topic = f"/q2r_{self.arm_name}_hand_pose" 
            self.quest_inputs_topic = f"/q2r_{self.arm_name}_hand_inputs" 
            # Command topic and gripper action topic
            self.robot_target_pose_topic = controller_prefix + topic_name_suffix
            self.gripper_action_topic = "/z1/gripper_action"

        else:
            self.get_logger().error(
            f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] "
            f"Unknown robot type: '{self.robot_type}'. Supported: 'franka', 'kuka', 'z1'."
        )
            raise ValueError(f"Unknown robot type: '{self.robot_type}'. Supported: 'franka', 'kuka', 'z1'.")
        
        # Summarize the configuration
        self.get_logger().info(
            f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] "
            f"Configured: base='{self.base_frame_id}', ee='{self.end_effector_link_name}', "
            f"target_topic='{self.robot_target_pose_topic}'"
            )
        
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
            self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 无法获取当前机器人变换: {e}")
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
                f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] "
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
                f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] "
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
                f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] "
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

        # rot_matrix_for_marker = tf_transformations.quaternion_matrix(q_quest_current_np)[:3,:3]
        # z_vec_marker = rot_matrix_for_marker.dot([0,0,1])
        # x_axis_marker = z_vec_marker / np.linalg.norm(z_vec_marker)

        # up_vec_marker = np.array([0.0, 0.0, 1.0])
        # if np.abs(np.dot(x_axis_marker, up_vec_marker)) > 0.9:
        #     up_vec_marker = np.array([0.0, 1.0, 0.0])

        # y_axis_marker = np.cross(up_vec_marker, x_axis_marker)
        # y_axis_marker /= np.linalg.norm(y_axis_marker)
        # z_axis_marker = np.cross(x_axis_marker, y_axis_marker)
        # z_axis_marker /= np.linalg.norm(z_axis_marker)

        # rot_matrix_final_marker = np.eye(4)
        # rot_matrix_final_marker[:3, 0] = x_axis_marker
        # rot_matrix_final_marker[:3, 1] = y_axis_marker
        # rot_matrix_final_marker[:3, 2] = z_axis_marker

        # q_marker = tf_transformations.quaternion_from_matrix(rot_matrix_final_marker)
        # q_marker /= np.linalg.norm(q_marker)
        # quat_marker = Quaternion(x=q_marker[0], y=q_marker[1], z=q_marker[2], w=q_marker[3])


        # # 1) 取当前末端位置（你前面刚算过 robot_pos）
        # cur_x, cur_y, cur_z = robot_pos  # robot_pos 来自 _get_robot_current_pose()

        # # 2) 取目标位置（你前面已生成 target_pose）
        # tgt_x = target_pose.position.x
        # tgt_y = target_pose.position.y
        # tgt_z = target_pose.position.z

        # # 3) 可选：计算并打印位移差/距离，方便调试
        # dx, dy, dz = (tgt_x - cur_x), (tgt_y - cur_y), (tgt_z - cur_z)
        # dist = (dx**2 + dy**2 + dz**2) ** 0.5


        
        # self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Δpos = ({dx:.3f}, {dy:.3f}, {dz:.3f}), |Δ| = {dist:.3f} m")

        # scale_factor = 10
        # dx *= scale_factor
        # dy *= scale_factor
        # dz *= scale_factor

        # end_x = cur_x + dx
        # end_y = cur_y + dy
        # end_z = cur_z + dz

        # # 4) 用两点模式画箭头：起点=当前，终点=目标
        # arrow = Marker()
        # arrow.header.frame_id = self.base_frame_id
        # arrow.header.stamp = self.get_clock().now().to_msg()
        # arrow.ns = f"{self.arm_name}_{self.robot_type}_current_to_target"
        # arrow.id = 100  # 避免与其他 marker 冲突
        # arrow.type = Marker.ARROW
        # arrow.action = Marker.ADD

        # # 两点（世界坐标，直接用 base_frame 下的坐标）
        # arrow.points = [Point(x=cur_x, y=cur_y, z=cur_z),
        #                 Point(x=end_x, y=end_y, z=end_z)]

        # # 两点模式下，scale 的含义：
        # #   x = 箭杆直径（shaft diameter）
        # #   y = 箭头锥体直径（head diameter）
        # #   z = 箭头长度（head length）
        # arrow.scale.x = 0.02  # 细一点的杆
        # arrow.scale.y = 0.05  # 箭头粗一点
        # arrow.scale.z = 0.01  # 箭头更明显

        # # 颜色：右臂红，左臂绿（与你之前一致）
        # arrow.color.r = 1.0 if self.arm_name == 'right' else 0.0
        # arrow.color.g = 0.0 if self.arm_name == 'right' else 1.0
        # arrow.color.b = 0.0
        # arrow.color.a = 1.0

        # self.marker_pub.publish(arrow)

        # # ============ 1️⃣ 位置点 ============
        # pos_marker = Marker()
        # pos_marker.header.frame_id = self.base_frame_id
        # pos_marker.header.stamp = self.get_clock().now().to_msg()
        # pos_marker.ns = f"{self.arm_name}_{self.robot_type}_target_pos"
        # pos_marker.id = 1
        # pos_marker.type = Marker.SPHERE
        # pos_marker.action = Marker.ADD
        # pos_marker.pose.position = target_pose.position
        # pos_marker.pose.orientation.w = 1.0  # 球体不需要旋转
        # pos_marker.scale.x = 0.03
        # pos_marker.scale.y = 0.03
        # pos_marker.scale.z = 0.03
        # pos_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # 橙色球体
        # self.marker_pub.publish(pos_marker)

        # ============ 2️⃣ 姿态坐标系 ============
        axes_marker = Marker()
        axes_marker.header.frame_id = self.base_frame_id
        axes_marker.header.stamp = self.get_clock().now().to_msg()
        axes_marker.ns = f"{self.arm_name}_{self.robot_type}_target_axes"
        axes_marker.id = 2
        axes_marker.type = Marker.LINE_LIST
        axes_marker.action = Marker.ADD
        axes_marker.pose.position = target_pose.position
        axes_marker.pose.orientation = target_pose.orientation
        axes_marker.scale.x = 0.02  # 线条粗细

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

        # ============ 3️⃣ 文本显示 xyz + xyzw ============
        # text_marker = Marker()
        # text_marker.header.frame_id = self.base_frame_id
        # text_marker.header.stamp = self.get_clock().now().to_msg()
        # text_marker.ns = f"{self.arm_name}_{self.robot_type}_target_text"
        # text_marker.id = 3
        # text_marker.type = Marker.TEXT_VIEW_FACING
        # text_marker.action = Marker.ADD
        # text_marker.pose.position.x = target_pose.position.x + 0.05
        # text_marker.pose.position.y = target_pose.position.y + 0.05
        # text_marker.pose.position.z = target_pose.position.z + 0.05
        # text_marker.scale.z = 0.05  # 字体高度
        # text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        # p = target_pose.position
        # o = target_pose.orientation

        # # ===== 全局固定参数 =====
        # fixed_x = 0.5   # 固定在 base frame 的这个位置（可调）
        # fixed_y = 0.0
        # fixed_z = 0.6   # 稍高一点，悬浮显示

        # dx_gap = 1.2  # 两列水平间距
        # font_h = 0.045  # 字体大小

        # # ---------- 左列：位置 (x, y, z) ----------
        # text_marker_pos = Marker()
        # text_marker_pos.header.frame_id = self.base_frame_id   # 固定 frame
        # text_marker_pos.header.stamp = self.get_clock().now().to_msg()
        # text_marker_pos.ns = "target_text_fixed"
        # text_marker_pos.id = 30
        # text_marker_pos.type = Marker.TEXT_VIEW_FACING
        # text_marker_pos.action = Marker.ADD

        # text_marker_pos.pose.position.x = fixed_x - dx_gap/2.0
        # text_marker_pos.pose.position.y = fixed_y
        # text_marker_pos.pose.position.z = fixed_z

        # text_marker_pos.scale.z = font_h
        # text_marker_pos.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        # text_marker_pos.text = (
        #     f"x={p.x:.3f}\n"
        #     f"y={p.y:.3f}\n"
        #     f"z={p.z:.3f}"
        # )
        # self.marker_pub.publish(text_marker_pos)

        # # ---------- 右列：姿态 (qx, qy, qz, qw) ----------
        # text_marker_ori = Marker()
        # text_marker_ori.header.frame_id = self.base_frame_id
        # text_marker_ori.header.stamp = self.get_clock().now().to_msg()
        # text_marker_ori.ns = "target_text_fixed"
        # text_marker_ori.id = 31
        # text_marker_ori.type = Marker.TEXT_VIEW_FACING
        # text_marker_ori.action = Marker.ADD

        # text_marker_ori.pose.position.x = fixed_x + dx_gap/2.0
        # text_marker_ori.pose.position.y = fixed_y
        # text_marker_ori.pose.position.z = fixed_z

        # text_marker_ori.scale.z = font_h
        # text_marker_ori.color = ColorRGBA(r=0.85, g=0.9, b=1.0, a=1.0)
        # text_marker_ori.text = (
        #     f"qx={o.x:.3f}\n"
        #     f"qy={o.y:.3f}\n"
        #     f"qz={o.z:.3f}\n"
        #     f"qw={o.w:.3f}"
        # )
        # self.marker_pub.publish(text_marker_ori)








        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = self.base_frame_id
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.pose = target_pose
        self.target_pose_publisher.publish(target_pose_stamped)
        self.last_executed_target_pose = target_pose
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Published new target pose.")
    
    def _toggle_gripper(self):
        """
        Toggle the gripper: send an action goal to open or close it depending on
        the current state. Updates internal state after the action completes.
        """
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Gripper action server not available.")
            return

        goal_msg = GripperCommand.Goal()

        if self.is_gripper_closed:
            goal_msg.command.position = 0.0 # 0.0 Fully open
            goal_msg.command.max_effort = 5.0 
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Sending goal to open gripper...")
        else:
            goal_msg.command.position = 0.05 # 0.05 Fully closed
            goal_msg.command.max_effort = 5.0 
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Sending goal to close gripper...")

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
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Upper button pressed. Toggling gripper...")
            self._toggle_gripper()
            
        # Update upper button press state
        self._button_upper_pressed_state = msg.button_upper

        # Lower button: Toggle pose updates and re-anchor
        if msg.button_lower and not self._button_lower_pressed_state:
            # Flip the pose-streaming gate
            self.allow_pose_update = not self.allow_pose_update
            
            state_msg = "ENABLED (publishing poses)" if self.allow_pose_update else "DISABLED (hold position)"
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Lower button pressed: pose streaming is now **{state_msg}**。")
            
            robot_pos, robot_ori = self._get_robot_current_pose()
            
            if self.last_pose_stamped_always is not None:
                pass

            if robot_pos is not None and robot_ori is not None:
                # Set robot anchor to the current EEF pose
                self.initial_position = robot_pos
                self.initial_orientation = robot_ori
                self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Robot anchor reset to current pose. ")
                
                # Force _pose_callback to re-anchor Quest on the next smoothed sample
                self.initial_orientation = None 
                self.first_received_quest_position = None
                self.first_received_quest_orientation = None

            # Clear filter history
            self.position_history.clear()
            self.orientation_history.clear()

        # Update lower button press state
        self._button_lower_pressed_state = msg.button_lower