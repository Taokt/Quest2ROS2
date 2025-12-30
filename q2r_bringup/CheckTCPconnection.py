import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from quest2ros.msg import OVR2ROSInputs # <--- DO NOT IMPORT THIS

class Listener(Node):
    def __init__(self):
        super().__init__("listener_node_py")
        
        # 1. SUBSCRIBE TO TWIST ONLY
        self.create_subscription(Twist, "q2r_right_hand_twist", self.twist_cb, 10)
        
        # 2. DO NOT SUBSCRIBE TO INPUTS
        # The crash happens because the Input message size is wrong, 
        # which corrupts the stream for the next message (Twist).
        # self.create_subscription(OVR2ROSInputs, ..., ...) 

    def twist_cb(self, msg):
        self.get_logger().info(f"Twist OK: x={msg.linear.x:.2f}")

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == "__main__":
    main()