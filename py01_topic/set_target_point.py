import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_action_client')

        self._client = ActionClient(self, MoveGroup, '/bh_robot/move_action')
        self._client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.create_request()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True

        self.get_logger().info('Sending MoveGroup goal...')
        self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb).add_done_callback(self.result_cb)

    def create_request(self):
        req = MotionPlanRequest()
        req.group_name = 'left_arm'

        # 设置目标姿态（注意 frame_id 和 link_name）
        pose = PoseStamped()
        pose.header.frame_id = 'left_base'
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.5  
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # 小 box 区域作为约束目标
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.1, 0.1]  # 非零小值

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = 'left_tool0'  # ⚠️ 替换为你的末端执行器 link 名称
        pc.constraint_region = bv
        pc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)

        req.goal_constraints.append(goal_constraints)
        req.start_state.is_diff = True

        return req

    def feedback_cb(self, feedback_msg):
        self.get_logger().info('Feedback received...')

    def result_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标被拒绝。')
            return

        self.get_logger().info('✅ 目标已接受，等待结果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.final_result_cb)

    def final_result_cb(self, future):
        result = future.result().result
        code = result.error_code.val
        if code == 1:
            self.get_logger().info('✅ 规划并执行成功！')
        else:
            self.get_logger().error(f'❌ 规划失败，错误码: {code}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveGroupClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
