import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Float64
#from moveit_commander import MoveGroupCommander
#from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from linkattacher_msgs.srv import AttachLink, DetachLink

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
    def __init__(self,point):
        super().__init__('go_to_pose_node')
        self.navigator = BasicNavigator()

        self.get_logger().info("⌛ Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 is active.")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # ✅ 来自 Gazebo 中的位置
        # goal_pose.pose.position.x = -0.584249
        # goal_pose.pose.position.y = -6.295790
        # goal_pose.pose.position.z = 0.0
        # roll = 0.000044
        # pitch = 0.000541
        # yaw = 0.089800
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.position.z = point[2]
        roll = point[3]
        pitch = point[4]
        yaw = point[5]
        # ✅ 欧拉角 → 四元数（Gazebo 中的角度）
       
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
            #self.grab_coke_can()  # 🎯 抵达目标后开始抓取
        else:
            self.get_logger().warn(f"❌ Navigation Failed: {result}")

class LinkAttacherClient(Node):
    def __init__(self):
        super().__init__('link_attacher_client')

    def attach(self, model1, link1, model2, link2):
        client = self.create_client(AttachLink, '/ATTACHLINK')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /ATTACHLINK 服务中...')
        request = AttachLink.Request()
        request.model1_name = model1
        request.link1_name = link1
        request.model2_name = model2
        request.link2_name = link2

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('附着成功')
        else:
            self.get_logger().error('附着失败')

    def detach(self, model1, link1, model2, link2):
        client = self.create_client(DetachLink, '/DETACHLINK')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /DETACHLINK 服务中...')
        request = DetachLink.Request()
        request.model1_name = model1
        request.link1_name = link1
        request.model2_name = model2
        request.link2_name = link2

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('释放成功')
        else:
            self.get_logger().error('释放失败')
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_trajectory_controller')
        self._client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_target_pose(self,joints):
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
            deg(joints[0]),
            deg(joints[1]),
            deg(joints[2]),
            deg(joints[3]),
            deg(joints[4]),
            deg(joints[5]),
            deg(joints[6])
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
        point.time_from_start.sec = 8
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
    #test(args)
    rclpy.init(args=args)
    

    # 修改为你的模型名和链接名
    robot_model = 'tiago'
    robot_link = 'gripper_right_finger_link'
    object_model = 'coke_can'
    object_link = 'link'

    point1 =[-0.584249,-6.295790,0.0,0.000044,0.000541,0.089800]
    point2 = [-2.2574231656455113,-6.363620554879826,0.0,0.0,0.0,-6.363620554879826
]
    point3 = [0.0,0.0,0.0,0.0,0.0,0.0]
    node = GoToPose(point1)
    rclpy.spin_once(node)
    
    time.sleep(4)

    node1 = ArmController()
    node1.send_target_pose([0,0,0,0,0,0,0])
    time.sleep(4)
    node1.send_target_pose([55,0,0,0,0,0,0])
    time.sleep(4)
    node1.send_target_pose([55,30,0,0,0,0,0])
    time.sleep(4)
    node1.send_target_pose([55,30,-90,0,0,0,0])
    time.sleep(4)
    node1.send_target_pose([55,30,-90,100,0,0,0])
    time.sleep(4)

    node2 = GripperController()
    # 🖐️ 打开夹爪（0.05）
    node2.send_gripper_goal(0.05)
    time.sleep(4)
    node1.send_target_pose([55,0,-90,100,0,0,0])
    time.sleep(4)
    # ✊ 关闭夹爪（0.035)
    node2.send_gripper_goal(0.035)
    time.sleep(4)
    # attach
    node3 = LinkAttacherClient()
    node3.attach(robot_model, robot_link, object_model, object_link)
    time.sleep(4)
    node1.send_target_pose([55,20,-90,100,0,0,0])
    time.sleep(4)
    
    node = GoToPose(point2)
    time.sleep(4)
    node = GoToPose(point3)

    node.destroy_node()
    node1.destroy_node()
    node2.destroy_node()
    node3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
