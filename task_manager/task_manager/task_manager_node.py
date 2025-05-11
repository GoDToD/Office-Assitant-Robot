import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess
import os

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.handle_command,
            10
        )
        self.get_logger().info("Task Manager initialized and waiting for commands.")

    def handle_command(self, msg):
        try:
            command = json.loads(msg.data)
            task = command.get("task")
            obj = command.get("object")
            location = command.get("location")

            self.get_logger().info(f"Received intent: task={task}, object={obj}, location={location}")

            # Run external task file if command is 'pick' and object is 'cola'
            if task == "pick" and obj == "cola":
                #self.get_logger().info("🧪 Running test task file for cola pickup...")
                subprocess.Popen(["python3", "/home/tod/tiago_public_ws/tasks.py"])
                return

            # Placeholder for other logic
            elif task == "go_to" and location:
                self.go_to_location(location)
            elif task == "deliver" and obj and location:
                self.deliver_object(obj, location)
            else:
                self.get_logger().warn(f"Unsupported or incomplete task: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse or handle command: {e}")

    def go_to_location(self, location):
        self.get_logger().info(f"🚶 Simulating navigation to: {location}")

    def pick_object(self, obj):
        self.get_logger().info(f"🤖 Simulating pickup of: {obj}")

    def deliver_object(self, obj, location):
        self.get_logger().info(f"📦 Simulating delivery of {obj} to {location}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
