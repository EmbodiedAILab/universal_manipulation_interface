import zmq
import pickle
import cv2
import time
import numpy as np

def start_subscriber():
    # 创建 ZeroMQ 套接字（SUB）
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")  # 连接到发布者

    # 订阅所有消息（空的订阅前缀表示接收所有消息）
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        # 接收发布者发送的数据
        serialized_data = socket.recv()

        # 反序列化接收到的数据
        cv_image = pickle.loads(serialized_data)

        # 获取接收时间并打印
        receive_time = time.time()
        print(f"Received image at {receive_time:.6f}")

        # 可视化图像
        cv2.imshow("Received Image", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_subscriber()
