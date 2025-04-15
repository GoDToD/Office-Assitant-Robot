import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time

from std_msgs.msg import Float64
#from moveit_commander import MoveGroupCommander
#from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose


# ✅ 不依赖任何外部库的欧拉角 → 四元数转换函数
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose_node')
        self.navigator = BasicNavigator()

        self.get_logger().info("⌛ Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 is active.")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # ✅ 来自 Gazebo 中的位置
        goal_pose.pose.position.x = -0.584249
        goal_pose.pose.position.y = -6.295790
        goal_pose.pose.position.z = 0.0

        # ✅ 欧拉角 → 四元数（Gazebo 中的角度）
        roll = 0.000044
        pitch = 0.000541
        yaw = 0.089800
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            self.get_logger().info("🛞 Moving...")
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ Navigation Success!")
            self.grab_coke_can()  # 🎯 抵达目标后开始抓取
        else:
            self.get_logger().warn(f"❌ Navigation Failed: {result}")


    # def grab_coke_can(self):
    #     self.get_logger().info("📦 Grabbing Coke can using moveit_py...")

    #     # ✅ 初始化 moveit_py
    #     moveit = MoveItPy(node=self)
    #     arm = moveit.get_planning_component("arm")
    #     gripper = moveit.get_planning_component("gripper")

    #     # ✅ 上方抓取位姿
    #     target_pose = Pose()
    #     target_pose.position.x = 0.395159
    #     target_pose.position.y = -6.163966
    #     target_pose.position.z = 0.95  # 抓取前稍高一点
    #     qx, qy, qz, qw = euler_to_quaternion(0.032540, -0.062413, -0.004006)
    #     target_pose.orientation.x = qx
    #     target_pose.orientation.y = qy
    #     target_pose.orientation.z = qz
    #     target_pose.orientation.w = qw

    #     arm.set_goal_state(pose=target_pose)
    #     plan = arm.plan()
    #     if plan:
    #         plan.execute()
    #     else:
    #         self.get_logger().error("❌ Failed to reach above the Coke can.")
    #         return

    #     # ✅ 向下靠近罐子
    #     target_pose.position.z = 0.80
    #     arm.set_goal_state(pose=target_pose)
    #     plan = arm.plan()
    #     if plan:
    #         plan.execute()
    #     else:
    #         self.get_logger().error("❌ Failed to move down to the Coke can.")
    #         return

    #     # ✅ 闭合夹爪（控制左指即可）
    #     gripper.set_goal_state(joint_positions={"gripper_left_finger_joint": 0.8})
    #     plan = gripper.plan()
    #     if plan:
    #         plan.execute()
    #         self.get_logger().info("🤏 Gripper closed.")
    #     else:
    #         self.get_logger().error("❌ Failed to close gripper.")
    #         return

    #     # ✅ 抬起可乐罐
    #     target_pose.position.z = 1.0
    #     arm.set_goal_state(pose=target_pose)
    #     plan = arm.plan()
    #     if plan:
    #         plan.execute()
    #         self.get_logger().info("✅ Can lifted successfully!")
    #     else:
    #         self.get_logger().error("❌ Failed to lift the can.")

    # def grab_coke_can(self):
    #     self.get_logger().info("📦 Starting to grab the Coke can...")

    #     # Step 1: MoveIt 机械臂控制器
    #     arm_group = MoveGroupCommander("arm")  # 请根据你 MoveIt 的配置修改 group 名称
    #     arm_group.set_max_velocity_scaling_factor(0.2)
    #     arm_group.set_max_acceleration_scaling_factor(0.2)

    #     # Step 2: 夹爪控制 publisher（假设是 Float64）
    #     gripper_pub = self.create_publisher(Float64, "/gripper_controller/command", 10)

    #     # Step 3: 设置抓取姿态（略高于目标点）
    #     target_pose = Pose()
    #     target_pose.position.x = 0.395159
    #     target_pose.position.y = -6.163966
    #     target_pose.position.z = 0.95  # 稍高于可乐罐上方
    #     qx, qy, qz, qw = euler_to_quaternion(0.032540, -0.062413, -0.004006)
    #     target_pose.orientation.x = qx
    #     target_pose.orientation.y = qy
    #     target_pose.orientation.z = qz
    #     target_pose.orientation.w = qw

    #     arm_group.set_pose_target(target_pose)
    #     success = arm_group.go(wait=True)
    #     arm_group.stop()
    #     arm_group.clear_pose_targets()

    #     if not success:
    #         self.get_logger().error("❌ Failed to reach above the Coke can.")
    #         return

    #     # Step 4: 下移靠近罐子
    #     target_pose.position.z = 0.80
    #     arm_group.set_pose_target(target_pose)
    #     success = arm_group.go(wait=True)
    #     arm_group.stop()
    #     arm_group.clear_pose_targets()

    #     if not success:
    #         self.get_logger().error("❌ Failed to move closer to the Coke can.")
    #         return

    #     # Step 5: 关闭夹爪抓取（1.0）
    #     gripper_msg = Float64()
    #     gripper_msg.data = 1.0
    #     gripper_pub.publish(gripper_msg)
    #     self.get_logger().info("🤏 Gripper closed to grab the can.")
    #     time.sleep(1.0)

    #     # Step 6: 抬起
    #     target_pose.position.z = 1.0
    #     arm_group.set_pose_target(target_pose)
    #     arm_group.go(wait=True)
    #     arm_group.stop()
    #     arm_group.clear_pose_targets()
    #     self.get_logger().info("✅ Coke can grabbed successfully!")
def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
