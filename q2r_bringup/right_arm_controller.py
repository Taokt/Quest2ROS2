import rclpy
from .robot_arm_controller_base import BaseArmController

class RightArmController(BaseArmController):
    def __init__(self, mirror=False):
        super().__init__(
            target_arm='right', # 'right' maps to Hugin (Physical Right Arm)
            mirror=mirror
        )

def main(args=None):
    rclpy.init(args=args)
    
    # Normal Mode: Right Hand -> Right Robot (Hugin)
    # Mirror Mode: Left Hand -> Right Robot (Hugin)
    node = RightArmController(mirror=True)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()