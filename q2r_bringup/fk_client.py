import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped

class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client_node')
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/bh_robot/joint_states',
            self.joint_state_callback,
            10
        )
        self.fk_client = self.create_client(GetPositionFK, '/bh_robot/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 FK 服务启动...')

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def get_fk(self):
        if not self.joint_state:
            self.get_logger().warn('尚未接收到关节状态数据')
            return

        request = GetPositionFK.Request()
        request.header.frame_id = 'bh_robot_base'
        request.fk_link_names = ['right_arm_link_7']  # 改成你想看的末端连杆
        request.robot_state.joint_state = self.joint_state

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            for pose_stamped in future.result().pose_stamped:
                self.get_logger().info(f'FK位置: x={pose_stamped.pose.position.x:.3f}, '
                                       f'y={pose_stamped.pose.position.y:.3f}, '
                                       f'z={pose_stamped.pose.position.z:.3f}')
        else:
            self.get_logger().error('FK服务调用失败')

def main(args=None):
    rclpy.init(args=args)
    node = FKClient()
    try:
        rclpy.spin_once(node, timeout_sec=2.0)  # 等待一次 joint_states 到来
        node.get_fk()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()