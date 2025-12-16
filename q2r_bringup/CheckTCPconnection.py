import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from quest2ros2_msg.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist

class Listener(Node):
    def __init__(self):
        super().__init__("listener_node_py")
        self.get_logger().info("creat the subscriber!")
        #self.subscription = self.create_subscription(Twist, "/q2r_right_hand_twist", self.do_cb,10)
        self.subscription = self.create_subscription(PoseStamped, "/q2r_right_hand_pose", self.do_cb,10)

    def do_cb(self,msg):
        #self.get_logger().info("subscribed data:%s"% msg.linear.x)
        # Access pose data from the message
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.get_logger().info(
            f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n"
            f"Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}"
        )
        


def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == "__main__":
    main()


