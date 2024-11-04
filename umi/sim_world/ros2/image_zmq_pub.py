import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import zmq
import cv2
import numpy as np
from cv_bridge import CvBridge
import pickle

class ZMQPublisher(Node):
    def __init__(self):
        super().__init__('zmq_publisher')

        # 创建 ZeroMQ 上下文和套接字
        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.PUB)
            self.socket.bind("tcp://*:5555")  # 绑定端口
        except Exception as e:
            self.get_logger().error(f"Error initializing ZeroMQ context: {e}")
            raise
        
        # 初始化 CVBridge 用于转换图像
        self.bridge = CvBridge()

        # 设置 topic 标签
        self.topic = "image_topic"  # 设置为具体的图像话题标签

        # 订阅 ROS 2 图像话题
        self.create_subscription(
            Image,
            '/test_image',  # 这里替换为你实际的图像话题
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # 将 ROS 2 图像消息转换为 OpenCV 图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f"Failed to convert image: {e}")
            return
        
        # 序列化图像数据
        serialized_data = pickle.dumps(cv_image)

        # 加上 topic 标签并发布图像数据
        try:
            # 使用 f-string 将 topic 标签添加到序列化的数据前
            message_with_topic = f"{self.topic} ".encode() + serialized_data
            self.socket.send(message_with_topic)
            self.get_logger().info(f"Published image with topic '{self.topic}'")
        except Exception as e:
            self.get_logger().error(f"Error sending data over ZeroMQ: {e}")
            raise

def main(args=None):
    rclpy.init(args=args)
    
    node = None  # 初始化 node 变量
    try:
        node = ZMQPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in node execution: {e}")
    finally:
        if node:
            node.destroy_node()  # 确保只有 node 初始化成功时才调用 destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
