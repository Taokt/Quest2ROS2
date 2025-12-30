import rclpy
from .robot_arm_controller_base import BaseArmController

class LeftArmController(BaseArmController):
    def __init__(self, mirror=False):
        super().__init__(
            target_arm='left',  # 'left' maps to Munin (Physical Left Arm)
            mirror=mirror       # Pass the mirror flag to the base class
        )

def main(args=None):
    rclpy.init(args=args)
    
    # Toggle mirror=True here if you want face-to-face control
    # Normal Mode: Left Hand -> Left Robot (Munin)
    # Mirror Mode: Right Hand -> Left Robot (Munin)
    node = LeftArmController(mirror=True)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()