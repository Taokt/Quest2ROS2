import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        # -------------------------------------------------------------
        # 1. CORRECT MAPPING: LEFT=MUNIN, RIGHT=HUGIN
        # -------------------------------------------------------------
        if target_arm == 'left':
            robot_name = "Munin"
            default_vr_hand = "left"   # Standard: Left Hand -> Left Robot (Munin)
        elif target_arm == 'right':
            robot_name = "Hugin"
            default_vr_hand = "right"  # Standard: Right Hand -> Right Robot (Hugin)
        else:
            raise ValueError(f"Unknown target_arm: {target_arm}. Expected 'left' or 'right'.")

        super().__init__(f"{robot_name.lower()}_bridge_node")

        # -------------------------------------------------------------
        # 2. HANDLE MIRRORING LOGIC
        # -------------------------------------------------------------
        # If mirror is ON, we listen to the OPPOSITE VR hand
        if mirror:
            source_hand = "right" if default_vr_hand == "left" else "left"
            self.get_logger().info(f"[{robot_name}] MIRROR MODE: Controlled by VR {source_hand.upper()} hand")
        else:
            source_hand = default_vr_hand
            self.get_logger().info(f"[{robot_name}] NORMAL MODE: Controlled by VR {source_hand.upper()} hand")

        # -------------------------------------------------------------
        # 3. CONFIGURE TOPICS
        # -------------------------------------------------------------
        # Input: Listen to the specific VR hand chosen above
        self.input_twist_topic = f"/q2r_{source_hand}_hand_twist"
        self.input_btn_topic = f"/q2r_{source_hand}_hand_inputs"
        
        # Output: Send to the standardized Robot Topic (/Munin/teleop_twist etc)
        self.output_cmd_topic = f"/{robot_name}/teleop_twist"
        
        # Gripper Action
        self.gripper_topic = f"/{robot_name}/gripper/gripper_action_controller/gripper_cmd"

        # -------------------------------------------------------------
        # 4. PARAMETERS & SUBSCRIBERS
        # -------------------------------------------------------------
        self.scale_lin = 1.0
        self.scale_ang = 1.0

        self.create_subscription(Twist, self.input_twist_topic, self._twist_cb, 10)
        self.create_subscription(OVR2ROSInputs, self.input_btn_topic, self._btn_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_topic)

        # State
        self.allow_motion = False
        self.gripper_closed = False
        self.last_upper = False

    def _btn_cb(self, msg):
        self.allow_motion = msg.button_lower
        if msg.button_upper and not self.last_upper:
            self._toggle_gripper()
        self.last_upper = msg.button_upper

    def _twist_cb(self, msg):
        cmd = Twist()
        if self.allow_motion:
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