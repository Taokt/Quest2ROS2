import rclpy
from .robot_arm_controller_base import BaseArmController

class LeftArmController(BaseArmController):
    def __init__(self):
        super().__init__(
            arm_name='right',
            robot_type='kuka' # Specify the robot type here
        )

def main(args=None):
    rclpy.init(args=args)
    node = LeftArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()