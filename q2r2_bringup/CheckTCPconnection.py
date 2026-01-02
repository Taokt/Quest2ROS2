"""
Quest Monitor Node
------------------
A diagnostic tool for the Quest-to-ROS bridge. This node actively listens to all standard 
controller topics (Pose, Inputs, Twist) for both Left and Right hands and prints a 
clean, formatted summary to the terminal at 1Hz.

Key Features:
- **Universal Listening**: Automatically subscribes to all 6 potential topic combinations 
  (Left/Right × Pose/Inputs/Twist).
- **Smart Display**: Only prints data for topics that are currently active, keeping the 
  terminal output clean.
- **Connection Watchdog**: Monitors data flow and alerts you if the connection is lost 
  (timeout > 2.0s) or if no topics are found.
- **Rate Limiting**: Buffers high-frequency incoming data but only prints updates once 
  per second to prevent terminal flooding.

How to Run:
    $ ros2 run q2r2_bringup CheckTCPconnection
    
Expected Output:
    --- Quest Status [Hour:Minute:Second] ---
    [LEFT_HAND_POSE]: Pos(0.50, 0.20, 0.30) | Ori(0.00, 0.00, 0.00, 1.00)
    [LEFT_HAND_INPUTS]: Press Index: True | Press Grip: False | Joystick: (0.50, -0.10)
"""
import rclpy
from rclpy.node import Node
import time

# Import message types
from geometry_msgs.msg import PoseStamped, Twist
from quest2ros.msg import OVR2ROSInputs

class QuestMonitor(Node):
    def __init__(self):
        super().__init__("quest_monitor_node")
        self.get_logger().info("--- Quest Monitor Started ---")
        self.get_logger().info("Waiting for Quest connection...")

        self.latest_data = {}
        self.last_msg_time = 0
        self.is_connected = False
        self.topic_types = {
            "pose": PoseStamped,
            "inputs": OVR2ROSInputs,
            "twist": Twist
        }
        self.sides = ["left", "right"]

        # Dynamically create subscribers for all 6 combinations
        self.subscriptions_list = []
        for side in self.sides:
            for name, msg_type in self.topic_types.items():
                topic_name = f"/q2r_{side}_hand_{name}"
                
                self.create_subscribed_callback(msg_type, topic_name)

        # Create a timer to print status every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def create_subscribed_callback(self, msg_type, topic_name):
        """Helper to create a subscriber with a specific topic name attached."""
        self.subscriptions_list.append(
            self.create_subscription(
                msg_type,
                topic_name,
                lambda msg: self.generic_callback(msg, topic_name),
                10
            )
        )

    def generic_callback(self, msg, topic_name):
        """Common callback for all topics."""
        self.last_msg_time = time.time()
        self.latest_data[topic_name] = msg
        self.is_connected = True

    def timer_callback(self):
        """Runs every 1 second to print the status."""
        current_time = time.time()
        time_since_last_msg = current_time - self.last_msg_time

        # Logic: If no message received for > 2.0 seconds, consider it disconnected
        if time_since_last_msg > 2.0:
            if self.is_connected:
                self.get_logger().error("Connect failed: Signal Lost (Timeout)")
                self.is_connected = False
            else:
                self.get_logger().warn("Connect failed: No active Quest topics found...")
            
            # Clear old data so we don't print stale values
            self.latest_data.clear()
            return

        # If connected, print the data
        print("\n" + "="*40)
        print(f"--- Quest Status [{time.strftime('%H:%M:%S')}] ---")
        
        # Sort keys to keep left/right grouped nicely in print output
        active_topics = sorted(self.latest_data.keys())

        for topic in active_topics:
            msg = self.latest_data[topic]
            self.print_formatted_msg(topic, msg)

    def print_formatted_msg(self, topic, msg):
        """Helper to format the output based on message type."""
        # Clean up topic name for display (remove leading slash)
        display_name = topic.replace("/q2r_", "").upper()

        if isinstance(msg, PoseStamped):
            p = msg.pose.position
            o = msg.pose.orientation
            print(f"[{display_name}]: Pos({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) | Ori({o.x:.2f}, {o.y:.2f}, {o.z:.2f}, {o.w:.2f})")
        
        elif isinstance(msg, Twist):
            l = msg.linear
            a = msg.angular
            print(f"[{display_name}]: Lin({l.x:.2f}, {l.y:.2f}, {l.z:.2f}) | Ang({a.x:.2f}, {a.y:.2f}, {a.z:.2f})")
        
        elif isinstance(msg, OVR2ROSInputs):
            try:
                # Example: printing trigger and grip which are common
                # Modify these fields based on your actual .msg definition
                print(f"[{display_name}]: Press Index: {msg.press_index} | Press Grip: {msg.press_grip} | Joystick: ({msg.thumbstick_x:.2f}, {msg.thumbstick_y:.2f})")
            except AttributeError:
                print(f"[{display_name}]: {msg}") # Fallback if fields don't match

def main():
    rclpy.init()
    node = QuestMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()