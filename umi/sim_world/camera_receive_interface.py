import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class CameraReceiver(Node):
    def __init__(self, topic_name):
        super().__init__('camera_receiver')
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.last_camera_data = {}

    def image_callback(self, msg):
        # 将ROS Image消息转换为OpenCV图像
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        print(f'received: {timestamp}')
        self.last_camera_data[timestamp] = {
            'color': image,
            'timestamp': timestamp
        }
        cv2.imshow("MetaEngine", image)
        cv2.waitKey(1)

    def get(self, k=None, out=None) -> dict:
        if out is None:
            out = dict()
        
        timestamps = sorted(self.last_camera_data.keys())[-k:]  # 获取最近的k个时间戳
        for i, timestamp in enumerate(timestamps):
            if i not in out:
                out[i] = {
                    'rgb': [],
                    'timestamp': []
                }
            image = self.last_camera_data[timestamp]['color']
            out[i]['rgb'].append(image)
            out[i]['timestamp'].append(timestamp)
        
        for i in out:
            out[i]['rgb'] = np.array(out[i]['rgb'])  # 转换为numpy数组，符合 (T, H, W, C) 的格式
            out[i]['timestamp'] = np.array(out[i]['timestamp'])
        
        print("Output 'out':", out)

        return out

def main(args=None):
    rclpy.init(args=args)
    topic_name = '/test_image'

    camera_receiver = CameraReceiver(topic_name)

    try:
        rclpy.spin_once(camera_receiver, timeout_sec=2.0)  # 等待2秒，确保接收到一些数据
        for i in range(10):
            # 检查是否有数据
            if camera_receiver.last_camera_data:
                camera_receiver.get(k=1)
            else:
                print("等待接收到图像数据...")
                time.sleep(1)  # 如果没有数据，等待1秒
    except KeyboardInterrupt:
        pass
    finally:
        camera_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()