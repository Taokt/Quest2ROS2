#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume, OrientationConstraint
from geometry_msgs.msg import Vector3, Pose
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener,LookupException, TransformException
import math
import sys
import time


def wait_for_tf(buffer, target_frame, source_frame, timeout_sec=2.0, poll_interval=0.1, node=None):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        try:
            now = node.get_clock().now().to_msg()
            if buffer.can_transform(target_frame, source_frame, now):
                return buffer.lookup_transform(target_frame, source_frame, now)
        except (LookupException, TransformException):
            pass
        time.sleep(poll_interval)
    if node:
        node.get_logger().error(f"Timeout: TF transform {target_frame} → {source_frame} not available after {timeout_sec}s.")
    return None

class MoveArmClient(Node):
    def __init__(self, group_name, link_name, frame_id):
        super().__init__('move_arm_client')
        self.group_name = group_name
        self.link_name = link_name
        self.frame_id = frame_id
        # 创建 MoveGroup 动作客户端，服务名为 /move_action
        self._action_client = ActionClient(self, MoveGroup, '/bh_robot/move_action')

        self.tf_buffer = Buffer()
        self.tf_lisnter = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'test1')

    def orientation_distance(self, q1, q2):
        dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
        dot = min(1.0, max(-1.0, dot))  # clamp
        angle = 2 * math.acos(abs(dot))
        return angle
    
    

    def send_goal(self, x, y, z):
        # 等待 action server 可用
        self._action_client.wait_for_server()
        self.get_logger().info(f'test2')

        # try:
        #     now = self.get_clock().now().to_msg()
        #     self.get_logger().info(f'test3')
        #     transform = wait_for_tf(self.tf_buffer, self.frame_id, self.link_name, timeout_sec=10.0, node=self)
        #     if not transform:
        #         self.get_logger().info(f'Not ready')
        #         return
        #     transform = self.tf_buffer.lookup_transform(
        #         self.frame_id,
        #         self.link_name,
        #         now,
        #         timeout=rclpy.duration.Duration(seconds=1.0)
        #     )
        #     current_pos = transform.transform.translation
        #     dx = x - current_pos.x
        #     dy = y - current_pos.y
        #     dz = z - current_pos.z
        #     dist = (dx**2 + dy**2 + dz**2)**0.5
        #     self.get_logger().info(f'Current position: ({current_pos.x:.3f}, {current_pos.y:.3f}, {current_pos.z:.3f})')
        #     self.get_logger().info(f'Distance to goal: {dist:.4f} m')

        #     if dist < 0.01:
        #         self.get_logger().info('Target position is already reached. Skipping planning.')
        #         return
            
        # except Exception as e:
        #     self.get_logger().error(f'Failed to get tf transform for {self.link_name}: {e}')
        #     return

        # 构造目标消息
        goal_msg = MoveGroup.Goal()
        # 设置规划组（左臂为例）
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 5.0
        # 构造目标约束
        pos_const = PositionConstraint()
        pos_const.header.frame_id = self.frame_id 
        pos_const.link_name = self.link_name
        pos_const.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 球半径 0.01 m
        bv.primitives.append(sphere)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.5
        bv.primitive_poses.append(pose)
        pos_const.constraint_region = bv
        pos_const.weight = 1.0

        # # 姿态约束
        # ori_const = OrientationConstraint()
        # ori_const.header.frame_id = self.frame_id 
        # ori_const.link_name = self.link_name
        # ori_const.orientation.x = 0.0
        # ori_const.orientation.y = 0.0
        # ori_const.orientation.z = 0.0
        # ori_const.orientation.w = 1.0
        # ori_const.absolute_x_axis_tolerance = 0.5
        # ori_const.absolute_y_axis_tolerance = 0.5
        # ori_const.absolute_z_axis_tolerance = 0.5
        # ori_const.weight = 1.0


        # combined constraint
        constraint = Constraints()
        constraint.position_constraints.append(pos_const)
        # constraint.orientation_constraints.append(ori_const)
        goal_msg.request.goal_constraints.append(constraint)

        # 设置规划选项：plan_only=False 表示规划后执行
        goal_msg.planning_options.plan_only = False
        # 发送目标并注册回调
        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={z}')
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

    def orientation_distance(q1, q2):
        dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
        dot = min(1.0, max(-1.0, dot))  # clamp
        angle = 2 * math.acos(abs(dot))
        return angle  # 弧度
    
   

# def main(args=None):
#     rclpy.init(args=args)
#     move_client = MoveArmClient()
#     # 直接内置目标位置，也可以改为从话题或服务获取
#     target_x = 0.4
#     target_y = 0.1
#     target_z = 0.4
#     move_client.send_goal(target_x, target_y, target_z)
#     rclpy.spin(move_client)

def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) != 5:
        print("Usage: ros2 run your_package your_script x y z [left|right]")
        return

    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_z = float(sys.argv[3])
    except ValueError:
        print("Please provide numeric values for x y z")
        return

    arm = sys.argv[4].lower()
    if arm == 'left':
        group_name = 'left_arm'
        frame_id = 'left_arm_link_0'
        link_name = 'left_arm_link_7'
    elif arm == 'right':
        group_name = 'right_arm_link_0'
        frame_id = 'right_arm_link_0'
        link_name = 'right_arm_link_7'
    else:
        print("Arm must be 'left' or 'right'")
        return

    move_client = MoveArmClient(group_name, link_name, frame_id)
    move_client.send_goal(target_x, target_y, target_z)
    rclpy.spin(move_client)

if __name__ == '__main__':
    main()








# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# from moveit_msgs.srv import GetCartesianPath
# from moveit_msgs.action import ExecuteTrajectory
# from rclpy.action import ActionClient
# import sys


# class CartesianPlannerExecutor(Node):
#     def __init__(self, group_name, link_name, frame_id):
#         super().__init__('cartesian_planner_executor')
#         self.group_name = group_name
#         self.link_name = link_name
#         self.frame_id = frame_id

#         self.cartesian_client = self.create_client(GetCartesianPath, '/bh_robot/compute_cartesian_path')
#         self.execute_client = ActionClient(self, ExecuteTrajectory, '/bh_robot/execute_trajectory')

#         while not self.cartesian_client.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn('Waiting for /compute_cartesian_path service...')

#         while not self.execute_client.wait_for_server(timeout_sec=2.0):
#             self.get_logger().warn('Waiting for /execute_trajectory action server...')

#     def plan_and_execute(self, x, y, z):
#         pose = Pose()
#         pose.position.x = x
#         pose.position.y = y
#         pose.position.z = z
#         pose.orientation.w = 1.0  # No rotation

#         request = GetCartesianPath.Request()
#         request.group_name = self.group_name
#         request.link_name = self.link_name
#         request.header.frame_id = self.frame_id
#         request.waypoints.append(pose)
#         request.max_step = 0.01
#         request.jump_threshold = 0.0
#         request.avoid_collisions = True

#         self.get_logger().info(f"Requesting Cartesian path to ({x}, {y}, {z})")
#         if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error('笛卡尔路径服务不可用！')
#             return
#         future = self.cartesian_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)

#         if not future.result():
#             self.get_logger().error('Cartesian planning service call failed')
#             return

#         result = future.result()
#         self.get_logger().info(f"Path planning fraction: {result.fraction:.2f}")

#         if result.fraction < 0.99:
#             self.get_logger().warn('Only part of the Cartesian path was planned. Not executing.')
#             return

#         goal = ExecuteTrajectory.Goal()
#         goal.trajectory = result.solution

#         self.get_logger().info("Sending trajectory for execution...")
#         future_exec = self.execute_client.send_goal_async(goal)
#         rclpy.spin_until_future_complete(self, future_exec)

#         goal_handle = future_exec.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Execution goal was rejected.')
#             return

#         self.get_logger().info('Execution goal accepted. Waiting for result...')
#         result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self, result_future)
#         result_exec = result_future.result().result
#         self.get_logger().info(f"Execution result code: {result_exec.error_code.val}")


# def main(args=None):
#     rclpy.init(args=args)

#     if len(sys.argv) != 5:
#         print("Usage: ros2 run py01_topic send_cartesian_execute x y z [left|right]")
#         return

#     x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
#     arm = sys.argv[4].lower()

#     if arm == 'left':
#         group_name = 'left_arm'
#         frame_id = 'left_arm_link_0'
#         link_name = 'left_arm_link_7'
#     elif arm == 'right':
#         group_name = 'right_arm'
#         frame_id = 'right_arm_link_0'
#         link_name = 'right_arm_link_7'
#     else:
#         print("Arm must be 'left' or 'right'")
#         return

#     node = CartesianPlannerExecutor(group_name, link_name, frame_id)
#     node.plan_and_execute(x, y, z)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
