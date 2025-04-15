import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_trajectory_controller')
        self._client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_target_pose(self):
        self.get_logger().info('⏳ Waiting for arm action server...')
        self._client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint'
        ]

        # 角度 → 弧度
        def deg(rad): return math.radians(rad)

        point = JointTrajectoryPoint()
        point.positions = [
            deg(69),
            deg(4),
            deg(-80),
            deg(44),
            deg(-80),
            deg(45),
            deg(-9)
        ]
        point.time_from_start.sec = 4
        goal_msg.trajectory.points = [point]

        self.get_logger().info('🎯 Sending joint trajectory goal...')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected by arm controller')
            return

        self.get_logger().info('✅ Goal accepted. Executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ Trajectory execution done!')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    node.send_target_pose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
