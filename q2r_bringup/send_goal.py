#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Vector3, Pose
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf2_geometry_msgs

import sys
import time

class MoveArmClient(Node):
    def __init__(self, group_name, link_name):
        super().__init__('move_arm_client')
        self.group_name = group_name
        self.link_name = link_name
        self._action_client = ActionClient(self, MoveGroup, '/bh_robot/move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def send_goal(self, x, y, z):
        self._action_client.wait_for_server()

        # 获取末端当前姿态（从 world 到 link_name）
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'world',
                self.link_name,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            orientation = transform.transform.rotation
            self.get_logger().info(f'Got current orientation: {orientation}')
        except Exception as e:
            self.get_logger().error(f'Failed to get current tf for {self.link_name}: {e}')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 5.0

        # 位置约束
        pos_const = PositionConstraint()
        pos_const.header.frame_id = 'world'
        pos_const.link_name = self.link_name
        pos_const.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose)

        pos_const.constraint_region = bv
        pos_const.weight = 1.0

        # 姿态约束（使用 tf 获取的当前姿态）
        ori_const = OrientationConstraint()
        ori_const.header.frame_id = 'world'
        ori_const.link_name = self.link_name
        ori_const.orientation = orientation
        ori_const.absolute_x_axis_tolerance = 0.1
        ori_const.absolute_y_axis_tolerance = 0.1
        ori_const.absolute_z_axis_tolerance = 0.1
        ori_const.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints.append(pos_const)
        constraint.orientation_constraints.append(ori_const)
        goal_msg.request.goal_constraints.append(constraint)

        goal_msg.planning_options.plan_only = False

        self.get_logger().info(f'Sending goal to {self.group_name}: x={x}, y={y}, z={z}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by server.')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('MoveGroup action succeeded.')
        else:
            self.get_logger().warning(f'MoveGroup action failed: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 5:
        print("Usage: ros2 run <your_package> send_goal_node x y z [left|right]")
        return

    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_z = float(sys.argv[3])
    except ValueError:
        print("x y z must be float values")
        return

    arm = sys.argv[4].lower()
    if arm == 'left':
        group_name = 'left_arm'
        link_name = 'left_arm_link_ee'
    elif arm == 'right':
        group_name = 'right_arm'
        link_name = 'right_arm_link_ee'
    else:
        print("Arm must be 'left' or 'right'")
        return

    move_client = MoveArmClient(group_name, link_name)
    move_client.send_goal(target_x, target_y, target_z)
    rclpy.spin(move_client)

if __name__ == '__main__':
    main()
