import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self, topic_name='/camera/color/image_raw'):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()

    def publish_frame(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)