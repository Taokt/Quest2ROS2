import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, TransformStamped
from quest2ros.msg import OVR2ROSInputs
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import numpy as np
import tf_transformations
import time # 考虑替换为 rclpy.time 以实现 ROS 感知的时间处理

# 导入 ROS 2 动作库和抓手动作消息
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class BaseArmController(Node):
    def __init__(self, arm_name: str, robot_type: str):
        """
        基于 Quest 3 输入控制机器人手臂的基类。

        参数:
            arm_name (str): 手臂名称（例如：'left'，'right'）。用于日志记录和区分。
            robot_type (str): 所控制的机器人类型（例如：'kuka'，'franka'，'z1'）。
        """
        # 我们将传递给父类构造函数的节点名称存储起来
        node_name_str = f"{arm_name}_{robot_type}_arm_controller"
        super().__init__(node_name_str)
        self._node_name = node_name_str # 将其存储在此处以供日志/引用

        self.arm_name = arm_name
        self.robot_type = robot_type

        # 根据类型和手臂名称配置机器人特定参数
        self._configure_robot_params()

        # 用于跟踪的状态变量
        self.last_pose_stamped_always = None
        self.initial_orientation = None # 机器人末端执行器的初始方向 (来自 TF)
        self.initial_position = None    # 机器人末端执行器的初始位置 (来自 TF)
        self.first_received_quest_position = None # 第一次接收到的 Quest 控制器位置
        self.first_received_quest_orientation = None # 第一次接收到的 Quest 控制器方向
        self.current_robot_pose = None  # 机器人末端执行器的当前位置 (来自 TF)
        self.allow_pose_update = True   # 标志，用于启用/禁用机器人移动 (来自 inputs_callback)

        # 创建订阅器
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.quest_pose_topic,
            self._pose_callback,
            10
        )
        self.input_subscription = self.create_subscription(
            OVR2ROSInputs,
            self.quest_inputs_topic,
            self._inputs_callback,
            10
        )

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, self.robot_target_pose_topic, 10)

        # 初始化 tf2 用于坐标转换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 准备接收 {self.quest_pose_topic} 并发布到 {self.robot_target_pose_topic}')
        # **修正行：使用存储的 _node_name 而不是 get_name()**
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 节点名称: {self._node_name}")

        # 用于记录轨迹的列表 (如果以后需要激活)
        self.commanded_trajectory_x = []
        self.commanded_trajectory_y = []
        self.commanded_trajectory_z = []
        self.executed_trajectory_x = []
        self.executed_trajectory_y = []
        self.executed_trajectory_z = []
        self.log_time = []
        # self.output_txt_name = f"outputs/{self.arm_name}_{self.robot_type}_trajectory_" + time.strftime("%Y%m%d-%H%M%S") + ".txt"
        # self.output_img_name = f"outputs/{self.arm_name}_{self.robot_type}_trajectory_" + time.strftime("%Y%m%d-%H%M%S") + ".png"
        
        # 抓手状态变量
        self.is_gripper_closed = False

        # 抓手动作客户端
        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_topic)
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 等待抓手动作服务器: {self.gripper_action_topic}")
        self.gripper_action_client.wait_for_server(timeout_sec=5.0)

    def _configure_robot_params(self):
        """
        根据机器人类型和手臂名称配置机器人特定参数（话题、TF 帧）。
        """
        topic_name_suffix = "/target_frame" # 目标位姿话题的通用后缀

        if self.robot_type == "franka":
            self.base_frame_id = "base"
            self.end_effector_link_name = "fr3_hand_tcp"
            controller_prefix = "/cartesian_impedance_controller"
            self.quest_pose_topic = f"/q2r_{self.arm_name}_hand_pose" # 假设 /q2r_left_hand_pose 或 /q2r_right_hand_pose
            self.quest_inputs_topic = f"/q2r_{self.arm_name}_hand_inputs"
            self.robot_target_pose_topic = controller_prefix + topic_name_suffix

        elif self.robot_type == "kuka":
            self.base_frame_id = "bh_robot_base" # KUKA 基准帧

            # 注意：您原始的 get_robot_params 中有一个关于 "lbr_link_0" 和 "lbr_link_ee" 的注释掉的部分。
            # 我假设您目前希望使用 "bh_robot_base" 和 "left_arm_link_ee"。
            # 如果您需要在不同的 KUKA 配置之间切换，可能需要添加另一个参数或逻辑。

            if self.arm_name == "left":
                self.end_effector_link_name = "left_arm_link_ee"
                controller_prefix = "/bh_robot/left_cartesian_impedance_controller"
                self.quest_pose_topic = "/q2r_left_hand_pose"
                self.quest_inputs_topic = "/q2r_left_hand_inputs"
                # 新增: 左臂抓手话题
                self.gripper_action_topic = "/bh_robot/left_arm_gripper_action_controller/gripper_cmd"
            elif self.arm_name == "right":
                # 确保您有这些右臂的实际对应项
                self.end_effector_link_name = "right_arm_link_ee" # 假设右臂末端执行器链接
                controller_prefix = "/bh_robot/right_cartesian_impedance_controller" # 假设右臂控制器
                self.quest_pose_topic = "/q2r_right_hand_pose"
                self.quest_inputs_topic = "/q2r_right_hand_inputs"
                # 新增: 右臂抓手话题
                self.gripper_action_topic = "/bh_robot/right_arm_gripper_action_controller/gripper_cmd"
            else:
                self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] KUKA 机器人未知的手臂名称: {self.arm_name}。期望为 'left' 或 'right'。")
                raise ValueError(f"KUKA 机器人未知的手臂名称: {self.arm_name}。期望为 'left' 或 'right'。")

            self.robot_target_pose_topic = controller_prefix + topic_name_suffix

        elif self.robot_type == "z1":
            self.base_frame_id = "link00"
            self.end_effector_link_name = "link06"
            controller_prefix = "/cartesian_impedance_controller"
            # 假设 Z1 可能没有针对 Quest 输入的独立左右手话题，或者它是一个单臂机器人
            self.quest_pose_topic = f"/q2r_{self.arm_name}_hand_pose" # 如果 Z1 有一个通用的手部姿态，请调整
            self.quest_inputs_topic = f"/q2r_{self.arm_name}_hand_inputs"
            self.robot_target_pose_topic = controller_prefix + topic_name_suffix

        else:
            self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 机器人类型未知: {self.robot_type}。支持的类型: 'franka'，'kuka'，'z1'。")
            raise ValueError(f"机器人类型未知: {self.robot_type}。支持的类型: 'franka'，'kuka'，'z1'。")

        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 配置完成: Base='{self.base_frame_id}'，EE='{self.end_effector_link_name}'，目标话题='{self.robot_target_pose_topic}'")

    def _get_robot_current_pose(self) -> (tuple | Quaternion | None):
        """
        尝试查找机器人末端执行器的当前位姿。
        返回 (x, y, z) 元组和 Quaternion 对象，如果变换失败则返回 None。
        """
        try:
            # 使用 rclpy.time.Time() 获取最新的可用变换
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.end_effector_link_name, rclpy.time.Time()
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_z = transform.transform.translation.z
            current_orientation = transform.transform.rotation
            self.current_robot_pose = (current_x, current_y, current_z)
            return (current_x, current_y, current_z), current_orientation
        except Exception as e:
            self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 无法获取当前机器人变换: {e}")
            return None, None

    def _pose_callback(self, pose_stamped: PoseStamped):
        """
        接收 Quest 3 手部姿态消息的回调函数。
        根据机器人初始姿态和 Quest 控制器偏移量计算目标姿态。
        """
        self.last_pose_stamped_always = pose_stamped

        if not self.allow_pose_update:
            # self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 不允许姿态更新。机器人移动暂停。")
            return # 抑制重复消息以避免日志泛滥

        # 1. 从 TF 获取机器人末端执行器的当前姿态
        robot_pos, robot_ori = self._get_robot_current_pose()
        if robot_pos is None or robot_ori is None:
            return # 未能获取机器人姿态，无法继续

        # 如果尚未设置初始机器人姿态，则进行设置
        if self.initial_orientation is None:
            self.initial_orientation = robot_ori
            self.initial_position = robot_pos
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 初始机器人姿态设置为: 位置={self.initial_position}，方向={self.initial_orientation}")

        # 2. 记录第一次接收到的 Quest 控制器姿态
        if self.first_received_quest_position is None:
            self.first_received_quest_position = pose_stamped.pose.position
            self.first_received_quest_orientation = pose_stamped.pose.orientation
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 第一次 Quest 姿态已记录。")
            return # 第一次调用只记录，不发布

        # 3. 计算来自 Quest 控制器的平移偏移量
        offset_x = pose_stamped.pose.position.x - self.first_received_quest_position.x
        offset_y = pose_stamped.pose.position.y - self.first_received_quest_position.y
        offset_z = pose_stamped.pose.position.z - self.first_received_quest_position.z

        # 4. 使用四元数数学计算旋转偏移量 (关键修正)
        q_quest_initial_np = np.array([self.first_received_quest_orientation.x, self.first_received_quest_orientation.y,
                                       self.first_received_quest_orientation.z, self.first_received_quest_orientation.w])
        q_quest_current_np = np.array([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                       pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])

        # 确保在操作前四元数已归一化
        # tf_transformations 函数通常会处理归一化，但明确归一化更安全。
        q_quest_initial_np = q_quest_initial_np / np.linalg.norm(q_quest_initial_np)
        q_quest_current_np = q_quest_current_np / np.linalg.norm(q_quest_current_np)

        # 计算从初始 Quest 方向到当前 Quest 方向的旋转
        # (q_relative = q_current * q_initial_inverse)
        # 四元数乘法顺序: q1 * q2 先应用 q2 再应用 q1 (从 q2 帧通过 q1 旋转)
        # 要从 A 旋转到 B: q_B * q_A_inverse
        q_quest_initial_inv = tf_transformations.quaternion_inverse(q_quest_initial_np)
        q_quest_relative_rotation = tf_transformations.quaternion_multiply(q_quest_current_np, q_quest_initial_inv)

        # 将相对 Quest 旋转应用于机器人初始方向
        # (q_target_robot = q_relative_rotation * q_robot_initial)
        # 将 Quest 控制器的“相对”旋转应用于机器人的“初始”方向。
        q_robot_initial_np = np.array([self.initial_orientation.x, self.initial_orientation.y,
                                       self.initial_orientation.z, self.initial_orientation.w])
        q_target_robot_np = tf_transformations.quaternion_multiply(q_quest_relative_rotation, q_robot_initial_np)
        q_target_robot_np = q_target_robot_np / np.linalg.norm(q_target_robot_np) # 归一化最终的四元数

        # 5. 将偏移量应用于机器人初始姿态
        target_pose = Pose()
        target_pose.position.x = self.initial_position[0] + offset_x
        target_pose.position.y = self.initial_position[1] + offset_y
        target_pose.position.z = self.initial_position[2] + offset_z

        target_pose.orientation.x = q_target_robot_np[0]
        target_pose.orientation.y = q_target_robot_np[1]
        target_pose.orientation.z = q_target_robot_np[2]
        target_pose.orientation.w = q_target_robot_np[3]

        # self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 目标姿态 (带偏移): 位置=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f}), 方向=({target_pose.orientation.x:.3f}, {target_pose.orientation.y:.3f}, {target_pose.orientation.z:.3f}, {target_pose.orientation.w:.3f})")

        # 6. 发布用于可视化的 Marker (可选)
        # 此 Marker 将显示 Quest 控制器“认为”其目标的位置，
        # 但其方向是根据 Quest 控制器的当前方向推导出来的。
        # 这是为了可视化而进行的近似。
        # Marker 的对齐方式遵循 Quest 当前方向相对于机器人基准帧的逻辑。
        rot_matrix_for_marker = tf_transformations.quaternion_matrix(q_quest_current_np)[:3,:3]
        z_vec_marker = rot_matrix_for_marker.dot([0,0,1])
        x_axis_marker = z_vec_marker / np.linalg.norm(z_vec_marker)

        # 鲁棒的“向上”向量选择，用于叉乘
        up_vec_marker = np.array([0.0, 0.0, 1.0])
        # 如果 x_axis_marker 与 Z 轴过于接近，则使用 Y 轴作为“向上”以避免共线
        if np.abs(np.dot(x_axis_marker, up_vec_marker)) > 0.9: # 如果点积接近 1 或 -1
            up_vec_marker = np.array([0.0, 1.0, 0.0]) # 使用 Y 轴作为备用“向上”

        y_axis_marker = np.cross(up_vec_marker, x_axis_marker)
        y_axis_marker /= np.linalg.norm(y_axis_marker) # 归一化
        z_axis_marker = np.cross(x_axis_marker, y_axis_marker)
        z_axis_marker /= np.linalg.norm(z_axis_marker) # 归一化 (应该已经正交，但仍是好习惯)


        rot_matrix_final_marker = np.eye(4)
        rot_matrix_final_marker[:3, 0] = x_axis_marker # Marker 的 X 轴与转换后的 Quest Z 轴对齐
        rot_matrix_final_marker[:3, 1] = y_axis_marker
        rot_matrix_final_marker[:3, 2] = z_axis_marker

        q_marker = tf_transformations.quaternion_from_matrix(rot_matrix_final_marker)
        # 确保 marker 四元数也已归一化
        q_marker = q_marker / np.linalg.norm(q_marker)
        quat_marker = Quaternion(x=q_marker[0], y=q_marker[1], z=q_marker[2], w=q_marker[3])

        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.arm_name}_{self.robot_type}_target_pose" # 每个手臂唯一的命名空间
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = target_pose.position # Marker 位置与目标姿态匹配
        marker.pose.orientation = quat_marker # Marker 方向遵循 Quest 控制器方向的逻辑以进行可视化

        marker.scale.x = 0.2
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        # 示例颜色区分: 右臂红色，左臂绿色
        marker.color.r = 1.0 if self.arm_name == 'right' else 0.0
        marker.color.g = 0.0 if self.arm_name == 'right' else 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        # 7. 将最终目标姿态发布到机器人
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = self.base_frame_id
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.pose = target_pose
        self.target_pose_publisher.publish(target_pose_stamped)
        self.last_executed_target_pose = target_pose
        self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] ✅ 已发布新的目标姿态。")

        # 可选：数据记录 (如果需要，请取消注释)
        # self.commanded_trajectory_x.append(target_pose.position.x)
        # self.commanded_trajectory_y.append(target_pose.position.y)
        # self.commanded_trajectory_z.append(target_pose.position.z)
        # self.executed_trajectory_x.append(robot_pos[0])
        # self.executed_trajectory_y.append(robot_pos[1])
        # self.executed_trajectory_z.append(robot_pos[2])
        # self.log_time.append(self.get_clock().now().nanoseconds / 1e9)


    def _toggle_gripper(self):
        """
        根据当前状态，发送动作目标以打开或关闭抓手。
        """
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 抓手动作服务器不可用。")
            return

        goal_msg = GripperCommand.Goal()

        if self.is_gripper_closed:
            # 打开抓手
            goal_msg.command.position = 0.0 # 0.0 代表完全打开
            goal_msg.command.max_effort = 5.0 # 最大努力值不应为 0
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 正在发送目标以打开抓手...")
        else:
            # 关闭抓手
            goal_msg.command.position = 0.05 # 0.05 代表完全关闭 (Robotiq Hand-E 典型值)
            goal_msg.command.max_effort = 5.0 # 设置为 10.0 以确保关闭
            self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 正在发送目标以关闭抓手...")

        # 发送目标并更新抓手状态
        self.gripper_action_client.send_goal_async(goal_msg)
        self.is_gripper_closed = not self.is_gripper_closed

    def _inputs_callback(self, msg: OVR2ROSInputs):
        """
        Quest 3 控制器输入消息的回调函数。
        通过按钮按下切换机器人移动并重置初始姿态。
        """
        # 如果按钮被按下，并且机器人当前正在移动...
        if msg.button_lower:
            if self.allow_pose_update: # 仅在状态从 True 变为 False 时触发
                self.allow_pose_update = False
                self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 已按下 Button_lower：机器人移动已暂停。")
                
                robot_pos, robot_ori = self._get_robot_current_pose()
                if robot_pos is not None and robot_ori is not None:
                    self.initial_position = robot_pos
                    self.initial_orientation = robot_ori
                    self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 机器人初始姿态已重置为当前位置。")
                else:
                    self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 无法获取机器人当前姿态以重置初始姿态。")
                
                if self.last_pose_stamped_always:
                    self.first_received_quest_position = self.last_pose_stamped_always.pose.position
                    self.first_received_quest_orientation = self.last_pose_stamped_always.pose.orientation
                    self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] Quest 新锚点已记录。")
                else:
                    self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 没有可用的Quest姿态。恢复控制可能出乎意料。")

        # 如果按钮被释放，并且机器人当前是暂停状态...
        else:
            if not self.allow_pose_update:
                self.allow_pose_update = True
                self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 已释放 Button_lower：机器人移动已恢复。")

        
        # 新增: 如果按下上部按钮，切换抓手状态
        if msg.button_upper:
            # 仅在按下按钮时触发一次
            if not hasattr(self, '_button_upper_pressed') or not self._button_upper_pressed:
                self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 检测到 Button_upper 按下。切换抓手状态...")
                self._toggle_gripper()
            self._button_upper_pressed = True
        else:
            self._button_upper_pressed = False
                
    # def _inputs_callback(self, msg: OVR2ROSInputs):
    #     """
    #     Quest 3 控制器输入消息的回调函数。
    #     通过按钮按下切换机器人移动并重置初始姿态。
    #     """
    #     if msg.button_lower:
    #         if self.allow_pose_update: # 仅在状态从 True 变为 False 时触发
    #             self.allow_pose_update = False
    #             self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 已按下 Button_lower：机器人移动已暂停。正在重置初始姿态。")
                
    #             # 暂停时将初始姿态重置为机器人当前姿态
    #             robot_pos, robot_ori = self._get_robot_current_pose()
    #             if robot_pos is not None and robot_ori is not None:
    #                 self.initial_orientation = robot_ori # 将机器人当前方向设置为新的初始方向
    #                 self.initial_position = robot_pos # 将机器人当前位置设置为新的初始位置
    #                 self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 新机器人起始姿态：位置={self.initial_position}，方向={self.initial_orientation}")
    #             else:
    #                 self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 无法获取机器人当前姿态以重置初始姿态。")
    #                 # 如果机器人姿态不可用，尝试使用最后接收到的 Quest 姿态作为初始姿态
    #                 # 这是一种备用方案，如果机器人不在 Quest 认为的位置，则可能不理想。
    #                 if self.last_pose_stamped_always:
    #                     self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 由于 TF 查找失败，正在使用最后 Quest 姿态作为初始姿态。")
    #                     self.first_received_quest_position = self.last_pose_stamped_always.pose.position
    #                     self.first_received_quest_orientation = self.last_pose_stamped_always.pose.orientation
    #                 else:
    #                     self.get_logger().error(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 没有可用的机器人姿态或 Quest 姿态来重置初始姿态。移动行为可能出乎意料。")

    #             # 暂停时始终将 Quest 的参考重置为最新的 Quest 姿态
    #             # 这确保了新的偏移量计算从暂停时 Quest 的位置开始。
    #             if self.last_pose_stamped_always:
    #                 self.first_received_quest_position = self.last_pose_stamped_always.pose.position
    #                 self.first_received_quest_orientation = self.last_pose_stamped_always.pose.orientation
    #                 self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 新 Quest 起始姿态：位置={self.first_received_quest_position}，方向={self.first_received_quest_orientation}")
    #             else:
    #                 self.get_logger().warn(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 没有可用的最后 Quest 姿态来重置 Quest 起始姿态。")

    #     else: # button_lower 为 False
    #         if not self.allow_pose_update: # 仅在状态从 False 变为 True 时触发
    #             self.allow_pose_update = True
    #             self.get_logger().info(f"[{self.arm_name.capitalize()} {self.robot_type.upper()} Arm] 已释放 Button_lower：机器人移动已恢复。")


    