import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__("listener_node_py")
        self.get_logger().info("creat the subscriber!")
        self.subscription = self.create_subscription(String, "chatter", self.do_cb,10)

    def do_cb(self,msg):
        self.get_logger().info("subscribed data:%s"% msg.data)


def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == "__main__":
    main()


