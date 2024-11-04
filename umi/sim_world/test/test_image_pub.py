import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'test_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每秒发布10帧
        self.bridge = CvBridge()

    def timer_callback(self):
        # 创建一个模拟图像（480x640, RGB格式）
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # 转换为ROS2的Image消息
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        # 发布图像消息
        self.publisher_.publish(img_msg)
        self.get_logger().info("Publishing image")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
