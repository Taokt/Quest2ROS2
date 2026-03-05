import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty  # <--- IMPORT 'Empty' HERE
from quest2ros.msg import OVR2ROSInputs
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class BaseArmController(Node):
    def __init__(self, target_arm: str, mirror: bool = False):
        """
        Args:
            target_arm (str): The physical ROBOT arm to control ('left' or 'right').
            mirror (bool): If True, swaps inputs (Right VR Hand -> Left Robot Arm).
        """
        if target_arm == 'left':
            robot_name = "Munin"
            default_vr_hand = "left"
        elif target_arm == 'right':
            robot_name = "Hugin"
            default_vr_hand = "right"
        else:
            raise ValueError(f"Unknown target_arm: {target_arm}. Expected 'left' or 'right'.")

        super().__init__(f"{robot_name.lower()}_bridge_node")

        if mirror:
            source_hand = "right" if default_vr_hand == "left" else "left"
            self.get_logger().info(f"[{robot_name}] MIRROR MODE: Controlled by VR {source_hand.upper()} hand")
        else:
            source_hand = default_vr_hand
            self.get_logger().info(f"[{robot_name}] NORMAL MODE: Controlled by VR {source_hand.upper()} hand")

        # --- TOPICS ---
        self.input_twist_topic = f"/q2r_{source_hand}_hand_twist"
        self.input_btn_topic = f"/q2r_{source_hand}_hand_inputs"
        
        self.output_cmd_topic = f"/{robot_name}/teleop_twist"
        self.gripper_topic = f"/{robot_name}/gripper/gripper_action_controller/gripper_cmd"
        self.home_topic = f"/{robot_name}/go_home"
        
        # NEW: Subtask topic (Global, not namespaced to specific arm so either hand can trigger it)
        self.subtask_topic = "/system/next_subtask"

        # --- PARAMETERS & SUBSCRIBERS ---
        self.scale_lin = 1.0
        self.scale_ang = 1.0
        self.deadman_threshold = 0.5

        # Debounce / Latching State
        self.debounce_duration = 0.3
        self.last_toggle_time = 0.0
        self.last_subtask_time = 0.0  # <--- NEW DEBOUNCE TIMER
        self.homing_btn_latched = False

        self.create_subscription(Twist, self.input_twist_topic, self._twist_cb, 10)
        self.create_subscription(OVR2ROSInputs, self.input_btn_topic, self._btn_cb, 10)
        
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.home_pub = self.create_publisher(Bool, self.home_topic, 10)
        
        # NEW PUBLISHER
        self.subtask_pub = self.create_publisher(Empty, self.subtask_topic, 10)
        
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_topic)

        # State
        self.allow_motion = False
        self.gripper_closed = False
        self.last_upper = False 
        self.last_lower = False  

    def _btn_cb(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 1. Deadman Switch
        self.allow_motion = (msg.press_middle > self.deadman_threshold)

        # 2. Homing Trigger Combo
        is_homing_combo = (msg.press_index > self.deadman_threshold) 
        if is_homing_combo:
            if not self.homing_btn_latched:
                self.get_logger().info(">>> VR HOMING TRIGGERED <<<")
                self.home_pub.publish(Bool(data=True))
                self.homing_btn_latched = True
        else:
            self.homing_btn_latched = False

        # 3. Gripper Toggle (Upper Button)
        if msg.button_upper and not self.last_upper:
            if (current_time - self.last_toggle_time) > self.debounce_duration:
                self._toggle_gripper()
                self.last_toggle_time = current_time
        self.last_upper = msg.button_upper

        # 4. SUBTASK SWITCH (Lower Button) <--- NEW LOGIC
        if msg.button_lower and not self.last_lower:
            if (current_time - self.last_subtask_time) > self.debounce_duration:
                self.get_logger().info(">>> SWITCHING TO NEXT SUBTASK <<<")
                self.subtask_pub.publish(Empty())
                self.last_subtask_time = current_time
        self.last_lower = msg.button_lower

    def _twist_cb(self, msg):
        cmd = Twist()
        if self.allow_motion and not self.homing_btn_latched:
            cmd.linear.x = msg.linear.x * self.scale_lin
            cmd.linear.y = msg.linear.y * self.scale_lin
            cmd.linear.z = msg.linear.z * self.scale_lin
            cmd.angular.x = msg.angular.x * self.scale_ang
            cmd.angular.y = msg.angular.y * self.scale_ang
            cmd.angular.z = msg.angular.z * self.scale_ang
        self.cmd_pub.publish(cmd)

    def _toggle_gripper(self):
        if not self.gripper_client.server_is_ready(): return
        goal = GripperCommand.Goal()
        goal.command.max_effort = 100.0
        
        if self.gripper_closed:
            goal.command.position = 0.0
            self.gripper_closed = False
        else:
            goal.command.position = 0.025
            self.gripper_closed = True
        
        self.gripper_client.send_goal_async(goal)