#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Quaternion
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Quaternion
from quest2ros.msg import OVR2ROSInputs
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import math
import numpy as np
import tf_transformations
import time



class RightHandFollower(Node):
    def __init__(self):
        super().__init__('right_hand_follower')

        self.busy = False
        topic = '/q2r_right_hand_pose'
        self.group_name = 'right_arm'
        self.link_name = 'right_arm_link_ee'
        self.frame_id = 'bh_robot_base'
        self.last_target_pose = None
        self.last_executed_target_pose = None
        

        self.initial_pose = None         # 初始点 
        self.first_received_position = None  # 第一次收到的位置（用于计算偏移）
        self.joint_state_stamp = None


        self.base_pose = None
        self.update_base_pose = False
        self.current_joint_state = None
        self.refe_pos_x = -0.8
        self.refe_pos_y = 0.43
        self.refe_pos_z = 0.76
        self.refe_pos_w = 1.0
        self.allow_pose_update = True
        self.joint_state_updated = True

        
        # 创建客户端
        self.cartesian_client = self.create_client(GetCartesianPath, '/bh_robot/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/bh_robot/execute_trajectory')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.fk_client = self.create_client(GetPositionFK, '/bh_robot/compute_fk')

        # 创建订阅器
        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            10
        )


        self.create_subscription(
            JointState,
            "/bh_robot/joint_states",
            self.joint_state_callback,
            10
        )

        self.input_subscription = self.create_subscription(
            OVR2ROSInputs,
            "/q2r_right_hand_inputs",
            self.inputs_callback,
            10
        )

        
        self.get_logger().info('waiting for service /bh_robot/compute_cartesian_path...')
        while not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('can not connected to /bh_robot/compute_cartesian_path')

        self.get_logger().info('waiting for Action Server /bh_robot/execute_trajectory...')
        while not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('can not connected to /bh_robot/execute_trajectory')

        self.get_logger().info('ready to receive /q2r_right_hand_pose target position')

        # 阻塞等待 joint state 和 FK 服务，然后设置 initial_pose
        self.get_logger().info("等待 joint state 和 FK 服务以设置初始位置...")

        while rclpy.ok():
            # 等到关节状态到位
            if self.current_joint_state is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 等到 FK 服务可用
            if not self.fk_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("FK 服务尚未就绪，继续等待...")
                continue

            # 构造 FK 请求
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.frame_id
            fk_request.fk_link_names = [self.link_name]
            fk_request.robot_state.joint_state = self.current_joint_state

            self.get_logger().info("发送 FK 请求以获取当前末端位姿...")
            future = self.fk_client.call_async(fk_request)

            # 等待 FK 返回
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            try:
                result = future.result()
                if not result.pose_stamped:
                    self.get_logger().error("FK 返回结果为空")
                    break

                fk_pose = result.pose_stamped[0].pose
                self.initial_pose = fk_pose
                self.initial_pose_for_reset = fk_pose
                self.get_logger().info("✅ 成功设置 initial_pose 为当前 FK 位置")
                break
            except Exception as e:
                self.get_logger().error(f"FK 调用失败: {e}")
                break



    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_updated = True
        self.joint_state_stamp = msg.header.stamp
        self.get_logger().info("✅ joint_state updated")
        

    

    def rotate_pose_around_z_axis(self, pose_stamped: PoseStamped, angle_deg: float):
        """绕 Z 轴顺时针旋转 PoseStamped 的 position 和 orientation"""
        theta = math.radians(angle_deg)

        # 构造旋转四元数
        half_theta = theta / 2.0
        rot_q = Quaternion()
        rot_q.z = math.sin(half_theta)
        rot_q.w = math.cos(half_theta)

        # 当前 orientation
        q1 = pose_stamped.pose.orientation
        q2 = rot_q

        # 四元数乘法
        qx = q2.w * q1.x + q2.x * q1.w + q2.y * q1.z - q2.z * q1.y
        qy = q2.w * q1.y - q2.x * q1.z + q2.y * q1.w + q2.z * q1.x
        qz = q2.w * q1.z + q2.x * q1.y - q2.y * q1.x + q2.z * q1.w
        qw = q2.w * q1.w - q2.x * q1.x - q2.y * q1.y - q2.z * q1.z

        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        return pose_stamped




    def pose_callback(self, pose_stamped):
        self.last_pose_stamped_always = pose_stamped

        if not self.allow_pose_update:
            # self.get_logger().info("当前禁止更新 target pose，忽略本次 pose")
            return

        if self.initial_pose is None:
            # self.get_logger().warn("Initial pose not ready yet, skipping this pose input.")
            return
        
         # 初始化固定参考初始点
        self.last_pose_stamped = pose_stamped  # 记录最近的 pose
        
        # 记录第一次收到的目标位姿
        if self.first_received_position is None:
            self.first_received_position = pose_stamped.pose.position
            # self.get_logger().info(f"First received position set to x={self.first_received_position.x:.3f}, "
            #                        f"y={self.first_received_position.y:.3f}, "
            #                        f"z={self.first_received_position.z:.3f}")
            return  # 第一次仅记录，不执行路径规划

        # 计算 offset = 当前Pose - 第一次Pose
        offset_x = pose_stamped.pose.position.x - self.first_received_position.x
        offset_y = pose_stamped.pose.position.y - self.first_received_position.y
        offset_z = pose_stamped.pose.position.z - self.first_received_position.z

        # 将 offset 应用到固定初始点
        target_pose = Pose()
        target_pose.position.x = self.initial_pose.position.x + offset_x
        target_pose.position.y = self.initial_pose.position.y + offset_y
        target_pose.position.z = self.initial_pose.position.z + offset_z

         # 设置方向
        orientation = pose_stamped.pose.orientation
        # orientation = rotated_pose.pose.orientation
        if orientation.x == 0.0 and orientation.y == 0.0 and orientation.z == 0.0 and orientation.w == 0.0:
            target_pose.orientation.w = 1.0
        else:
            target_pose.orientation = orientation

        self.get_logger().info(f"Target pose (with offset): x={target_pose.position.x:.3f}, "
                               f"y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")



        rot = tf_transformations.quaternion_matrix([
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        ])[:3,:3]

        z_vec = rot.dot([0,0,1])
        x_axis = z_vec / np.linalg.norm(z_vec)

        # 3. 构造与 x_axis 不共线的 up 向量
        up = np.array([0.0, 0.0, 1.0]) if abs(x_axis[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y_axis = np.cross(up, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)

        # 4. 构造朝向 z_vec 的旋转矩阵（x_axis 对齐 z_vec）
        rot_matrix = np.eye(4)
        rot_matrix[:3, 0] = x_axis
        rot_matrix[:3, 1] = y_axis
        rot_matrix[:3, 2] = z_axis

        # 5. 转为四元数
        q = tf_transformations.quaternion_from_matrix(rot_matrix)
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = q

  
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_pose"
        marker.id = 0
        marker.type = Marker.ARROW  
        marker.action = Marker.ADD
        marker.pose.position = target_pose.position  # 包含 position + orientation
        marker.pose.orientation = quat

        # 尺寸设置（箭头长度=0.2，箭身宽=0.04，箭头宽=0.04）
        marker.scale.x = 0.2  # 箭头指向方向（length）
        marker.scale.y = 0.04  # 箭头宽度（shaft width）
        marker.scale.z = 0.04  # 箭头高度（head width）

        # 红色透明箭头
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)



        # ✅ 新增：若当前目标和上次成功执行位置相差很小，则跳过
        if self.last_executed_target_pose and not self.is_pose_significantly_different(target_pose, self.last_executed_target_pose, threshold=0.02):
            # self.get_logger().debug("目标位置与上次执行几乎一致，跳过执行")
            return

        if self.busy:
            # self.get_logger().warn("Moving not finished, ignore request")
            return
        self.busy = True



        # 构建路径请求
        request = GetCartesianPath.Request()
        request.group_name = self.group_name
        request.link_name = self.link_name
        request.header.frame_id = self.frame_id
        request.waypoints.append(target_pose)
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.joint_state = self.current_joint_state
        request.start_state.is_diff = True



        

        # self.get_logger().info("Calling cartesian_server...")
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            # self.get_logger().error('cartesian_server no response')
            self.busy = False
            return

        self.last_target_pose = target_pose  # ⚠️ 必须更新这次的目标位姿

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self.cartesian_response_callback)


    def try_initialize_from_fk(self):
        if self.fk_initialized:
            return

        if self.current_joint_state is None:
            # self.get_logger().warn("Waiting for joint state to initialize initial_pose...")
            return

        if not self.fk_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().warn("FK service not ready yet")
            return

        # self.get_logger().info("Sending FK request to initialize initial_pose...")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = self.frame_id
        fk_request.fk_link_names = [self.link_name]
        fk_request.robot_state.joint_state = self.current_joint_state

        future = self.fk_client.call_async(fk_request)

        def fk_callback(fut):
            if fut.result() is None:
                # self.get_logger().error(f"Initial FK failed: {fut.exception()}")
                return

            result = fut.result()
            if not result.pose_stamped:
                # self.get_logger().error("Initial FK returned empty result")
                return

            fk_pose = result.pose_stamped[0].pose
            self.initial_pose = fk_pose
            self.initial_pose_for_reset = fk_pose
            # self.get_logger().info("✅ Automatically set initial_pose from current FK pose")

            self.fk_initialized = True

        future.add_done_callback(fk_callback)


    def is_pose_significantly_different(self, pose1, pose2, threshold=0.08):
        dx = abs(pose1.position.x - pose2.position.x)
        dy = abs(pose1.position.y - pose2.position.y)
        dz = abs(pose1.position.z - pose2.position.z)

        result = (dx + dy + dz) > threshold


        self.get_logger().info(
            f"Pose1: x={pose1.position.x:.3f}, y={pose1.position.y:.3f}, z={pose1.position.z:.3f} | "
            f"Pose2: x={pose2.position.x:.3f}, y={pose2.position.y:.3f}, z={pose2.position.z:.3f} | "
            f"Delta: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}"
            f"result:{result}"
        )

        return result


    def inputs_callback(self, msg: OVR2ROSInputs):
        if msg.button_upper:
            # self.get_logger().info("button_upper pressed — returning to initial position")

            if self.initial_pose is None:
                # self.get_logger().warn("Initial pose not set yet, ignoring return request")
                return

            if self.last_pose_stamped is None:
                # self.get_logger().warn("No pose received yet, can't update reference point")
                return

            if self.busy:
                # self.get_logger().warn("Robot is busy, ignoring return-to-initial request")
                return

            # 更新参考点为当前 PoseStamped（下次 offset 计算会以它为新基准）
            self.first_received_position = self.last_pose_stamped.pose.position
            # self.get_logger().info("Updated first_received_position as new reference")

            self.initial_pose = self.initial_pose_for_reset

            # 构建回初始点的请求
            request = GetCartesianPath.Request()
            request.group_name = self.group_name
            request.link_name = self.link_name
            request.header.frame_id = self.frame_id
            request.waypoints.append(self.initial_pose_for_reset)
            request.max_step = 0.01
            request.jump_threshold = 0.0
            request.avoid_collisions = True

            self.busy = True
            # self.get_logger().info("Sending trajectory to initial position")

            future = self.cartesian_client.call_async(request)
            future.add_done_callback(self.cartesian_response_callback)
        
        if msg.button_lower:
            self.allow_pose_update = False
            # self.get_logger().info("button_lower 被按下，设置当前位置为新的初始点")

            if self.busy:
                # self.get_logger().warn("机器人正忙，忽略请求")
                return

            if self.current_joint_state is None:
                # self.get_logger().warn("尚未接收到关节状态，无法计算 FK")
                return

            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.frame_id
            fk_request.fk_link_names = [self.link_name]
            fk_request.robot_state.joint_state = self.current_joint_state

            future = self.fk_client.call_async(fk_request)

            def fk_callback(fut):
                if fut.result() is None:
                    # self.get_logger().error(f"调用 compute_fk 失败: {fut.exception()}")
                    return

                result = fut.result()
                if not result.pose_stamped:
                    # self.get_logger().error("FK 结果为空")
                    return

                fk_pose = result.pose_stamped[0].pose
                self.initial_pose = fk_pose
                # self.first_received_position = self.last_pose_stamped.pose.position if self.last_pose_stamped else None
                self.first_received_position = self.last_pose_stamped_always.pose.position if self.last_pose_stamped_always else None
                # self.get_logger().info("已设置新的初始位置为 FK 结果")

            future.add_done_callback(fk_callback)
        else:
            self.allow_pose_update = True


    def cartesian_response_callback(self, future):
        if future.result() is None:
            # self.get_logger().error(f"cartesian_server calling failed，exception: {future.exception()}")
            self.busy = False
            return

        result = future.result()
        # self.get_logger().info(f'Percentage of planning（fraction）: {result.fraction:.3f}')

        if result.fraction < 0.99:
            # self.get_logger().error('Planned path is not complete,cancelling execute')
            self.busy = False
            return

        # 构造轨迹执行请求
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution

        # self.get_logger().info('sending executing requet..')

        try:
            send_future = self.execute_client.send_goal_async(goal)
        except Exception as e:
            # self.get_logger().error(f"发送 execute_trajectory 异步目标失败: {e}")
            self.busy = False
            return

        def wrapped_exec_callback(fut):
            # self.get_logger().info("进入 execute_done_callback")
            try:
                goal_handle = fut.result()
            except Exception as e:
                # self.get_logger().error(f"获取 goal_handle 失败: {e}")
                self.busy = False
                return

            if not goal_handle.accepted:
                # self.get_logger().error('执行请求被拒绝')
                self.busy = False
                return

            # self.get_logger().info('轨迹已被接受，等待执行完成...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execution_result_callback)

        send_future.add_done_callback(wrapped_exec_callback)


    def execute_done_callback(self, future):

        # self.get_logger().info("进入 execute_done_callback")
        try:
            goal_handle = future.result()
        except Exception as e:
            # self.get_logger().error(f"获取 goal_handle 失败: {e}")
            self.busy = False
            return

        if not goal_handle.accepted:
            # self.get_logger().error('执行请求被拒绝')
            self.busy = False
            return

        # self.get_logger().info('轨迹已被接受，等待执行完成...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execution_result_callback)


    def execution_result_callback(self, future):
        # self.get_logger().info("进入 execution_result_callback")
        try:
            result = future.result().result
            # self.get_logger().info(f"执行完成，返回码: {result.error_code.val}")
            self.last_executed_target_pose = self.last_target_pose
        except Exception as e:
            self.get_logger().error(f"获取执行结果失败: {e}")

        
        self.busy = False


# --- NEW: LeftHandFollower Class ---
class LeftHandFollower(Node):
    def __init__(self):
        super().__init__('left_hand_follower') # Different node name

        self.busy = False
        topic = '/q2r_left_hand_pose' # IMPORTANT: Different topic for left hand
        self.group_name = 'left_arm' # IMPORTANT: Left arm group name (e.g., 'left_arm')
        self.link_name = 'left_arm_link_ee' # IMPORTANT: Left arm end-effector link (e.g., 'left_arm_link_ee')
        self.frame_id = 'bh_robot_base' # Base frame is common
        self.last_target_pose = None
        self.last_executed_target_pose = None

        self.initial_pose = None
        self.first_received_position = None
        self.joint_state_stamp = None

        self.base_pose = None
        self.update_base_pose = False
        self.current_joint_state = None # Still subscribing to the same joint states
        # self.refe_pos_x, self.refe_pos_y, self.refe_pos_z, self.refe_pos_w might need adjustment for left arm's typical pose if different
        self.allow_pose_update = True
        self.joint_state_updated = True
        self.fk_initialized = False


        # Create clients (common for both arms, as MoveIt! serves both)
        self.cartesian_client = self.create_client(GetCartesianPath, '/bh_robot/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/bh_robot/execute_trajectory')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.fk_client = self.create_client(GetPositionFK, '/bh_robot/compute_fk')

        # Create subscribers
        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            10
        )

        # JointState subscription is common, but it's important that this node
        # also gets the latest joint states. For simplicity, both nodes can subscribe.
        # Alternatively, you could have a single "robot state monitor" node.
        self.create_subscription(
            JointState,
            "/bh_robot/joint_states",
            self.joint_state_callback,
            10
        )

        self.input_subscription = self.create_subscription(
            OVR2ROSInputs, # IMPORTANT: You might need a separate topic for left hand inputs
            "/q2r_left_hand_inputs", # Example: assuming a separate input topic
            self.inputs_callback,
            10
        )

        self.get_logger().info('Left arm: waiting for service /bh_robot/compute_cartesian_path...')
        while not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Left arm: can not connected to /bh_robot/compute_cartesian_path')

        self.get_logger().info('Left arm: waiting for Action Server /bh_robot/execute_trajectory...')
        while not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Left arm: can not connected to /bh_robot/execute_trajectory')

        self.get_logger().info('Left arm: ready to receive /q2r_left_hand_pose target position')

        self.get_logger().info("Left arm: 等待 joint state 和 FK 服务以设置初始位置...")
        while rclpy.ok() and not self.fk_initialized:
            if self.current_joint_state is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            if not self.fk_client.wait_for_service(timeout_sec=1.0):
                # self.get_logger().warn("Left arm: FK 服务尚未就绪，继续等待...")
                continue

            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.frame_id
            fk_request.fk_link_names = [self.link_name]
            fk_request.robot_state.joint_state = self.current_joint_state

            # self.get_logger().info("Left arm: 发送 FK 请求以获取当前末端位姿...")
            future = self.fk_client.call_async(fk_request)

            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            try:
                result = future.result()
                if not result.pose_stamped:
                    # self.get_logger().error("Left arm: FK 返回结果为空")
                    break

                fk_pose = result.pose_stamped[0].pose
                self.initial_pose = fk_pose
                self.initial_pose_for_reset = fk_pose
                # self.get_logger().info("✅ Left arm: 成功设置 initial_pose 为当前 FK 位置")
                self.fk_initialized = True
                break
            except Exception as e:
                # self.get_logger().error(f"Left arm: FK 调用失败: {e}")
                break

    # The following methods are largely identical to RightHandFollower,
    # but with log messages adjusted to indicate 'Left arm'.
    # Ensure any hardcoded values or references specific to 'right' are changed.

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_updated = True
        self.joint_state_stamp = msg.header.stamp
        # self.get_logger().info("✅ Left joint_state updated")

    def rotate_pose_around_z_axis(self, pose_stamped: PoseStamped, angle_deg: float):
        """绕 Z 轴顺时针旋转 PoseStamped 的 position 和 orientation"""
        theta = math.radians(angle_deg)

        half_theta = theta / 2.0
        rot_q = Quaternion()
        rot_q.z = math.sin(half_theta)
        rot_q.w = math.cos(half_theta)

        q1 = pose_stamped.pose.orientation
        q2 = rot_q

        qx = q2.w * q1.x + q2.x * q1.w + q2.y * q1.z - q2.z * q1.y
        qy = q2.w * q1.y - q2.x * q1.z + q2.y * q1.w + q2.z * q1.x
        qz = q2.w * q1.z + q2.x * q1.y - q2.y * q1.x + q2.z * q1.w
        qw = q2.w * q1.w - q2.x * q1.x - q2.y * q1.y - q2.z * q1.z

        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        return pose_stamped

    def pose_callback(self, pose_stamped):
        self.last_pose_stamped_always = pose_stamped

        if not self.allow_pose_update:
            # self.get_logger().info("Left arm: 当前禁止更新 target pose，忽略本次 pose")
            return

        if self.initial_pose is None:
            # self.get_logger().warn("Left arm: Initial pose not ready yet, skipping this pose input.")
            return

        self.last_pose_stamped = pose_stamped

        if self.first_received_position is None:
            self.first_received_position = pose_stamped.pose.position
            # self.get_logger().info(f"Left arm: First received position set to x={self.first_received_position.x:.3f}, "
            #                        f"y={self.first_received_position.y:.3f}, "
            #                        f"z={self.first_received_position.z:.3f}")
            return

        offset_x = pose_stamped.pose.position.x - self.first_received_position.x
        offset_y = pose_stamped.pose.position.y - self.first_received_position.y
        offset_z = pose_stamped.pose.position.z - self.first_received_position.z

        target_pose = Pose()
        target_pose.position.x = self.initial_pose.position.x + offset_x
        target_pose.position.y = self.initial_pose.position.y + offset_y
        target_pose.position.z = self.initial_pose.position.z + offset_z

        orientation = pose_stamped.pose.orientation
        if orientation.x == 0.0 and orientation.y == 0.0 and orientation.z == 0.0 and orientation.w == 0.0:
            target_pose.orientation.w = 1.0
        else:
            target_pose.orientation = orientation

        # self.get_logger().info(f"Left arm: Target pose (with offset): x={target_pose.position.x:.3f}, "
        #                        f"y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")

        rot = tf_transformations.quaternion_matrix([
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        ])[:3,:3]

        z_vec = rot.dot([0,0,1])
        x_axis = z_vec / np.linalg.norm(z_vec)

        up = np.array([0.0, 0.0, 1.0]) if abs(x_axis[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y_axis = np.cross(up, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)

        rot_matrix = np.eye(4)
        rot_matrix[:3, 0] = x_axis
        rot_matrix[:3, 1] = y_axis
        rot_matrix[:3, 2] = z_axis

        q = tf_transformations.quaternion_from_matrix(rot_matrix)
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = q

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_pose_left" # Different namespace for left arm marker
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = target_pose.position
        marker.pose.orientation = quat

        marker.scale.x = 0.2
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0 # Blue for left arm
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        if self.last_executed_target_pose and not self.is_pose_significantly_different(target_pose, self.last_executed_target_pose, threshold=0.02):
            # self.get_logger().debug("Left arm: 目标位置与上次执行几乎一致，跳过执行")
            return

        if self.busy:
            # self.get_logger().warn("Left arm: Moving not finished, ignore request")
            return
        self.busy = True

        request = GetCartesianPath.Request()
        request.group_name = self.group_name
        request.link_name = self.link_name
        request.header.frame_id = self.frame_id
        request.waypoints.append(target_pose)
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.joint_state = self.current_joint_state
        request.start_state.is_diff = True

        # self.get_logger().info("Left arm: Calling cartesian_server...")
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            # self.get_logger().error('Left arm: cartesian_server no response')
            self.busy = False
            return

        self.last_target_pose = target_pose

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self.cartesian_response_callback)

    def is_pose_significantly_different(self, pose1, pose2, threshold=0.08):
        dx = abs(pose1.position.x - pose2.position.x)
        dy = abs(pose1.position.y - pose2.position.y)
        dz = abs(pose1.position.z - pose2.position.z)

        result = (dx + dy + dz) > threshold
        # self.get_logger().info(
        #     f"Left arm: Pose1: x={pose1.position.x:.3f}, y={pose1.position.y:.3f}, z={pose1.position.z:.3f} | "
        #     f"Pose2: x={pose2.position.x:.3f}, y={pose2.position.y:.3f}, z={pose2.position.z:.3f} | "
        #     f"Delta: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f} | "
        #     f"Result: {result}"
        # )
        return result

    def inputs_callback(self, msg: OVR2ROSInputs):
        if msg.button_upper:
            # self.get_logger().info("Left arm: button_upper pressed — returning to initial position")

            if self.initial_pose is None:
                # self.get_logger().warn("Left arm: Initial pose not set yet, ignoring return request")
                return

            if self.last_pose_stamped is None:
                # self.get_logger().warn("Left arm: No pose received yet, can't update reference point")
                return

            if self.busy:
                # self.get_logger().warn("Left arm: Robot is busy, ignoring return-to-initial request")
                return

            self.first_received_position = self.last_pose_stamped.pose.position
            # self.get_logger().info("Left arm: Updated first_received_position as new reference")
            self.initial_pose = self.initial_pose_for_reset

            request = GetCartesianPath.Request()
            request.group_name = self.group_name
            request.link_name = self.link_name
            request.header.frame_id = self.frame_id
            request.waypoints.append(self.initial_pose_for_reset)
            request.max_step = 0.01
            request.jump_threshold = 0.0
            request.avoid_collisions = True

            self.busy = True
            # self.get_logger().info("Left arm: Sending trajectory to initial position")

            future = self.cartesian_client.call_async(request)
            future.add_done_callback(self.cartesian_response_callback)

        if msg.button_lower:
            self.allow_pose_update = False
            # self.get_logger().info("Left arm: button_lower 被按下，设置当前位置为新的初始点")

            if self.busy:
                # self.get_logger().warn("Left arm: 机器人正忙，忽略请求")
                return

            if self.current_joint_state is None:
                # self.get_logger().warn("Left arm: 尚未接收到关节状态，无法计算 FK")
                return

            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.frame_id
            fk_request.fk_link_names = [self.link_name]
            fk_request.robot_state.joint_state = self.current_joint_state

            future = self.fk_client.call_async(fk_request)

            def fk_callback(fut):
                if fut.result() is None:
                    # self.get_logger().error(f"Left arm: 调用 compute_fk 失败: {fut.exception()}")
                    return

                result = fut.result()
                if not result.pose_stamped:
                    # self.get_logger().error("Left arm: FK 结果为空")
                    return

                fk_pose = result.pose_stamped[0].pose
                self.initial_pose = fk_pose
                self.first_received_position = self.last_pose_stamped_always.pose.position if self.last_pose_stamped_always else None
                # self.get_logger().info("Left arm: 已设置新的初始位置为 FK 结果")

            future.add_done_callback(fk_callback)
        else:
            self.allow_pose_update = True


    def cartesian_response_callback(self, future):
        if future.result() is None:
            # self.get_logger().error(f"Left arm: cartesian_server calling failed，exception: {future.exception()}")
            self.busy = False
            return

        result = future.result()
        # self.get_logger().info(f'Left arm: Percentage of planning（fraction）: {result.fraction:.3f}')

        if result.fraction < 0.99:
            # self.get_logger().error('Left arm: Planned path is not complete,cancelling execute')
            self.busy = False
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution

        # self.get_logger().info('Left arm: sending executing requet..')

        try:
            send_future = self.execute_client.send_goal_async(goal)
        except Exception as e:
            # self.get_logger().error(f"Left arm: 发送 execute_trajectory 异步目标失败: {e}")
            self.busy = False
            return

        def wrapped_exec_callback(fut):
            # self.get_logger().info("Left arm: 进入 execute_done_callback")
            try:
                goal_handle = fut.result()
            except Exception as e:
                # self.get_logger().error(f"Left arm: 获取 goal_handle 失败: {e}")
                self.busy = False
                return

            if not goal_handle.accepted:
                # self.get_logger().error('Left arm: 执行请求被拒绝')
                self.busy = False
                return

            # self.get_logger().info('Left arm: 轨迹已被接受，等待执行完成...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execution_result_callback)

        send_future.add_done_callback(wrapped_exec_callback)


    def execution_result_callback(self, future):
        # self.get_logger().info("Left arm: 进入 execution_result_callback")
        try:
            result = future.result().result
            # self.get_logger().info(f"Left arm: 执行完成，返回码: {result.error_code.val}")
            self.last_executed_target_pose = self.last_target_pose
        except Exception as e:
            self.get_logger().error(f"Left arm: 获取执行结果失败: {e}")
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = LeftHandFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # rclpy.init(args=args)
    # right_hand_node = RightHandFollower()
    # left_hand_node = LeftHandFollower() # Instantiate the left hand controller

    # # Use a MultiThreadedExecutor to allow both nodes to run concurrently
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(left_hand_node)
    # executor.add_node(right_hand_node)


    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     executor.shutdown()
    #     right_hand_node.destroy_node()
    #     left_hand_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
