import zmq
import cv2
import time
import numpy as np
import pickle
import threading

class CameraReceiver:
    def __init__(self, zmq_host="localhost", zmq_port=5555, topic="image_topic"):
        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.topic = topic
        self.last_camera_data = {}

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)  # 订阅指定的 topic

        self.lock = threading.Lock()

    def receive_images(self):
        """
        从 ZeroMQ 订阅图像数据，并将其存储到 last_camera_data 中。
        """
        while True:
            try:
                # 接收带 topic 前缀的数据
                message = self.socket.recv()
                topic, serialized_data = message.split(b" ", 1)  # 分割出 topic 和数据
                receive_time = time.time()

                # 反序列化数据为 OpenCV 图像
                cv_image = pickle.loads(serialized_data)

                # 存储接收到的数据
                timestamp = int(receive_time * 1e6) 
                with self.lock:  # 使用锁来确保线程安全
                    self.last_camera_data[timestamp] = {
                        'color': cv_image,
                        'timestamp': receive_time
                    }

                # 显示接收到的图像
                cv2.imshow("Received Image", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print(f"An error occurred while receiving data: {e}")
                break

        cv2.destroyAllWindows()

    def get(self, k=None, out=None) -> dict:
        """
        返回与 'camera.get()' 函数兼容的格式数据。
        """
        if out is None:
            out = dict()
        
        with self.lock:  # 使用锁来确保线程安全
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

        return out

def main():
    camera_receiver = CameraReceiver(zmq_host="localhost", zmq_port=5555, topic="image_topic")

    # 在一个单独的线程中运行 receive_images
    receive_thread = threading.Thread(target=camera_receiver.receive_images)
    receive_thread.daemon = True  # 将线程设置为守护线程，程序退出时自动退出
    receive_thread.start()

    try:
        while True:
            latest_data = camera_receiver.get(k=5)
            print("Latest received images:", latest_data)
            time.sleep(1) 
    except KeyboardInterrupt:
        print("Terminating the server.")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
