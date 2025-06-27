##control left arm
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker
from quest2ros.msg import OVR2ROSInputs

class LeftHandFollower(Node):
    def __init__(self):
        super().__init__('left_hand_follower')

        self.busy = False
        self.initial_pose = None
        self.first_received_position = None
        self.last_pose_stamped = None

        self.topic = '/q2r_left_hand_pose'
        self.input_topic = '/q2r_left_hand_inputs'
        self.group_name = 'left_arm'
        self.link_name = 'left_arm_link_7'
        self.frame_id = 'bh_robot_base'

        self.cartesian_client = self.create_client(GetCartesianPath, '/bh_robot/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/bh_robot/execute_trajectory')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.subscription = self.create_subscription(PoseStamped, self.topic, self.pose_callback, 10)
        self.input_subscription = self.create_subscription(OVR2ROSInputs, self.input_topic, self.inputs_callback, 10)

        self.get_logger().info('waiting for service /bh_robot/compute_cartesian_path...')
        while not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('can not connected to /bh_robot/compute_cartesian_path')

        self.get_logger().info('waiting for Action Server /bh_robot/execute_trajectory...')
        while not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('can not connected to /bh_robot/execute_trajectory')

        self.get_logger().info('ready to receive /q2r_left_hand_pose target position')

    def pose_callback(self, pose_stamped):
        self.last_pose_stamped = pose_stamped

        if self.initial_pose is None:
            self.initial_pose = Pose()
            self.initial_pose.position.x = -0.6
            self.initial_pose.position.y = 0.05
            self.initial_pose.position.z = 0.285
            self.initial_pose.orientation.w = 1.0

        if self.first_received_position is None:
            self.first_received_position = pose_stamped.pose.position
            self.get_logger().info(f"First received position set to x={self.first_received_position.x:.3f}, "
                                   f"y={self.first_received_position.y:.3f}, "
                                   f"z={self.first_received_position.z:.3f}")
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

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "left_target_pose"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target_pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        if self.busy:
            self.get_logger().warn("Moving not finished, ignore request")
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

        self.get_logger().info("Calling cartesian_server...")
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('cartesian_server no response')
            self.busy = False
            return

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self.cartesian_response_callback)

    def inputs_callback(self, msg: OVR2ROSInputs):
        if msg.button_upper:
            self.get_logger().info("button_upper pressed — returning to initial position")

            if self.initial_pose is None:
                self.get_logger().warn("Initial pose not set yet, ignoring return request")
                return

            if self.last_pose_stamped is None:
                self.get_logger().warn("No pose received yet, can't update reference point")
                return

            if self.busy:
                self.get_logger().warn("Robot is busy, ignoring return-to-initial request")
                return

            self.first_received_position = self.last_pose_stamped.pose.position
            self.get_logger().info("Updated first_received_position as new reference")

            request = GetCartesianPath.Request()
            request.group_name = self.group_name
            request.link_name = self.link_name
            request.header.frame_id = self.frame_id
            request.waypoints.append(self.initial_pose)
            request.max_step = 0.01
            request.jump_threshold = 0.0
            request.avoid_collisions = True

            self.busy = True
            self.get_logger().info("Sending trajectory to initial position")

            future = self.cartesian_client.call_async(request)
            future.add_done_callback(self.cartesian_response_callback)

    def cartesian_response_callback(self, future):
        if future.result() is None:
            self.get_logger().error(f"cartesian_server calling failed, exception: {future.exception()}")
            self.busy = False
            return

        result = future.result()
        self.get_logger().info(f'Percentage of planning: {result.fraction:.3f}')

        if result.fraction < 0.99:
            self.get_logger().warn('Planned path is not complete, cancelling execute')
            self.busy = False
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution

        self.get_logger().info('Sending execution request...')
        send_future = self.execute_client.send_goal_async(goal)
        send_future.add_done_callback(self.execute_done_callback)

    def execute_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Executing request was rejected')
            self.busy = False
            return

        self.get_logger().info('Planning result accepted, waiting for execution to finish')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execution_result_callback)

    def execution_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Execution finished, return code: {result.error_code.val}")
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = LeftHandFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

