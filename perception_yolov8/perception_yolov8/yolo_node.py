import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import atexit

class Yolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.latest_frame = None
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # Update path/model name if needed
        self.model.names[39] = "Cola Can"
		
        self.get_logger().info("YOLOv8 node initialized.")
        atexit.register(cv2.destroyAllWindows)
        
        self.timer = self.create_timer(0.5, self.run_detection)
        
    def image_callback(self, msg):
        self.latest_frame = msg
        
    def run_detection(self):
        if self.latest_frame is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
            results = self.model(frame)
            results[0].names[39] = "Cola Can"
            annotated = results[0].plot()
            cv2.imshow("YOLOv8 Detection", annotated)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
