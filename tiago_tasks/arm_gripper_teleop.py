import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
import time

ARM_JOINTS = [
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
    'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
]

GRIPPER_JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.current_arm_positions = [0.0] * 7
        self.gripper_open = True
        self.step_size = 0.1

    def send_arm_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = self.current_arm_positions
        point.time_from_start.sec = 2
        goal_msg.trajectory.points = [point]

        self.arm_client.wait_for_server()
        self.arm_client.send_goal_async(goal_msg)
        self.get_logger().info(f"🦾 Sent arm goal: {self.current_arm_positions}")

    def toggle_gripper(self, open_gripper: bool):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        pos = 0.0 if open_gripper else 0.035
        point.positions = [pos, pos]
        point.time_from_start.sec = 1
        goal_msg.trajectory.points = [point]

        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal_msg)
        self.get_logger().info(f"{'🖐️ Open' if open_gripper else '✊ Close'} gripper")

def keyboard_loop(stdscr, node: ArmTeleop):
    stdscr.nodelay(True)
    key_map = {
        ord('q'): 0, ord('a'): 0,
        ord('w'): 1, ord('s'): 1,
        ord('e'): 2, ord('d'): 2,
        ord('r'): 3, ord('f'): 3,
        ord('t'): 4, ord('g'): 4,
        ord('y'): 5, ord('h'): 5,
        ord('u'): 6, ord('j'): 6,
    }
    sign_map = {
        ord('q'): 1, ord('a'): -1,
        ord('w'): 1, ord('s'): -1,
        ord('e'): 1, ord('d'): -1,
        ord('r'): 1, ord('f'): -1,
        ord('t'): 1, ord('g'): -1,
        ord('y'): 1, ord('h'): -1,
        ord('u'): 1, ord('j'): -1,
    }

    stdscr.addstr(0, 0, "🎮 Keyboard teleop: q/a w/s e/d ... u/j for joints | o=open | c=close | space=execute | ESC=exit")

    while True:
        key = stdscr.getch()
        if key == 27:  # ESC
            break
        elif key in key_map:
            idx = key_map[key]
            delta = sign_map[key] * node.step_size
            node.current_arm_positions[idx] += delta
            stdscr.addstr(2 + idx, 0, f"{ARM_JOINTS[idx]}: {node.current_arm_positions[idx]:.2f}         ")
        elif key == ord(' '):
            node.send_arm_goal()
        elif key == ord('o'):
            node.toggle_gripper(True)
        elif key == ord('c'):
            node.toggle_gripper(False)
        time.sleep(0.1)

def main():
    rclpy.init()
    node = ArmTeleop()
    curses.wrapper(keyboard_loop, node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
