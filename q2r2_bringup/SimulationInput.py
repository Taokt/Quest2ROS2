import rclpy
from rclpy.node import Node
import numpy as np
import time

from quest2ros.msg import OVR2ROSInputs
from geometry_msgs.msg import PoseStamped, Twist

class QuestSimulator(Node):
    def __init__(self):
        super().__init__("quest_simulator_node")
        
        # declaration default mode: all and right
        self.declare_parameter('mode', 'all')  
        self.declare_parameter('side', 'right') 
        
        self.gripper_state = False
        self.last_toggle_time = time.time()

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.side = self.get_parameter('side').get_parameter_value().string_value
        
        self.get_logger().info(f"--- Simulator Started | Side: {self.side.upper()} | Mode: {self.mode.upper()} ---")
        
        # topic name
        pose_topic = f"/q2r_{self.side}_hand_pose"
        inputs_topic = f"/q2r_{self.side}_hand_inputs"
        twist_topic = f"/q2r_{self.side}_hand_twist"

        # === Initialize publisher ===
        if self.mode in ['teleop', 'all']:
            self.inputs_pub = self.create_publisher(OVR2ROSInputs, inputs_topic, 10)
            self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        
        if self.mode in ['velocity', 'all']:
            self.twist_pub = self.create_publisher(Twist, twist_topic, 10)

        self.timer = self.create_timer(1.0/30.0, self.on_timer)
        self.start_time = time.time()

    def on_timer(self):
        now_time = time.time()
        now_ros = self.get_clock().now().to_msg()
        elapsed = time.time() - self.start_time

        # === publish Inputs and Pose ===
        if self.mode in ['teleop', 'all']:
            inputs_msg = OVR2ROSInputs()
            inputs_msg.button_lower = True 
            
            # change Upper button
            if now_time - self.last_toggle_time > 5.0:
                self.gripper_state = not self.gripper_state
                self.last_toggle_time = now_time
                self.get_logger().info(f"Simulating button_upper toggle: {self.gripper_state}")
            
            inputs_msg.button_upper = self.gripper_state
            self.inputs_pub.publish(inputs_msg)

            # drawing circle
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now_ros
            pose_msg.header.frame_id = "world"
            y_offset = 0.2 if self.side == 'left' else -0.2
            pose_msg.pose.position.x = 0.5 + 0.1 * np.cos(elapsed)
            pose_msg.pose.position.y = y_offset + 0.1 * np.sin(elapsed)
            pose_msg.pose.position.z = 0.5
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)

        # === publish Twist ===
        if self.mode in ['velocity', 'all']:
            twist_msg = Twist()
            twist_msg.linear.x = -0.1 * np.sin(elapsed) # 对应 x 位置的导数
            self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuestSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



