#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2
from moveit2.trajectory_planner import TrajectoryPlanner

class MoveToXYZ(Node):
    def __init__(self, group_name: str, target_xyz: list):
        super().__init__('move_to_xyz')
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[],  # 可以不填，交给 MoveIt 自动识别
            base_link_name="left_arm_link_0",
            end_effector_name="left_arm_link_ee",
            group_name=group_name,
            execute=True,
            planner_id="RRTConnectkConfigDefault",
        )

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = target_xyz[0]
        pose.pose.position.y = target_xyz[1]
        pose.pose.position.z = target_xyz[2]
        pose.pose.orientation.w = 1.0  # 默认单位四元数（朝向不变）

        self.moveit2.move_to_pose(pose)

def main():
    import sys
    if len(sys.argv) < 5:
        print("Usage: ros2 run <your_pkg> go_to_xyz.py <group_name> x y z")
        return

    rclpy.init()
    group = sys.argv[1]
    x, y, z = map(float, sys.argv[2:5])
    node = MoveToXYZ(group, [x, y, z])
    rclpy.spin(node)

if __name__ == '__main__':
    main()

