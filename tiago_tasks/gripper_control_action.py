import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_action_controller')
        self._client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def send_gripper_goal(self, position: float):
        self.get_logger().info('⏳ Waiting for gripper action server...')
        self._client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'gripper_right_finger_joint',
            'gripper_left_finger_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'🚀 Sending goal: position = {position}')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected by gripper controller')
            return

        self.get_logger().info('✅ Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code == 0:
            self.get_logger().info('✅ Gripper action succeeded!')
        else:
            self.get_logger().error(f'❌ Gripper action failed with error code: {result.error_code}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()

    # 🖐️ 打开夹爪（0.0）
    node.send_gripper_goal(0.0)
    time.sleep(2)
    # ✊ 关闭夹爪（0.035）
    node.send_gripper_goal(0.035)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
