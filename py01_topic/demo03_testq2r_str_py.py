import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from base_interfaces_demo.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist

class Talker(Node):
    def __init__(self):
        super().__init__("talker_node_py")
        self.get_logger().info("creat the talker!")
        self.right_hand_inputs_publisher = self.create_publisher(OVR2ROSInputs,"q2r_right_hand_inputs",10)
        self.right_hand_twist_publisher = self.create_publisher(Twist,"/q2r_right_hand_twist",10)

        self.timer = self.create_timer(1.0,self.on_timer)

    def on_timer(self):
        right_inputs_message = OVR2ROSInputs()
        right_inputs_message.button_upper = False
        right_inputs_message.button_lower = True
        right_inputs_message.thumb_stick_horizontal = 1.5
        right_inputs_message.thumb_stick_vertical = 2.0
        right_inputs_message.press_index = 3.2
        right_inputs_message.press_middle = 1.1
        self.right_hand_inputs_publisher.publish(right_inputs_message)
        self.get_logger().info("published right_inputs data: %d"% right_inputs_message.button_lower) 


        right_twist_message = Twist()
        # linear
        right_twist_message.linear.x = 0.2  
        right_twist_message.linear.y = 0.0  
        right_twist_message.linear.z = 0.0  
        # angular
        right_twist_message.angular.x = 0.0  
        right_twist_message.angular.y = 0.0  
        right_twist_message.angular.z = 0.5  
        self.right_hand_twist_publisher.publish(right_twist_message)
        self.get_logger().info("published right_twist data: %f"% right_twist_message.linear.x) 

        


def main():
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()
