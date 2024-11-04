import zmq
import cv2
import time
import numpy as np
import pickle
import threading

HOST = "192.168.1.103" 

class CameraReceiver:
    def __init__(self, zmq_host=HOST, zmq_port=5553, topic="image_topic"):
        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.topic = topic.encode()
        self.last_camera_data = {}

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
        self.socket.setsockopt(zmq.SUBSCRIBE, self.topic)

        self.lock = threading.Lock()

    def receive_images(self, callback):
        """
        从 ZeroMQ 订阅图像数据，并将其存储到 last_camera_data 中。
        """
        t_start = time.time()
        while True:
            try:
                message = self.socket.recv()
                topic, serialized_data = message.split(b" ", 1)
                receive_time = time.time()

                # topic, serialized_data = self.socket.recv_multipart()
                # receive_time = time.time()

                if topic == self.topic:
                    cv_image = pickle.loads(serialized_data)

                    # timestamp = int(receive_time * 1e6)
                    data ={
                            'rgb': cv_image,
                            'timestamp': receive_time
                    }

                    # cv2.imshow("Received Image", cv_image)
                    # if cv2.waitKey(1) & 0xFF == ord('q'):
                    #         break

                    callback(data)
                    t_end = time.time()
                    duration = t_end - t_start
                    frequency = np.round(1 / duration, 1)
                    print(f'FPS {frequency}, duration: {t_end- receive_time}, recv latency: {receive_time-t_start}')
                    t_start = t_end
                    # with self.lock:
                    #     self.last_camera_data[timestamp] = {
                    #         'color': cv_image,
                    #         'timestamp': receive_time
                    #     }                    

            except Exception as e:
                print(f"An error occurred while receiving data: {e}")
                break

#    def get(self, k=None, out=None) -> dict:
#        """
#        返回与 'camera.get()' 函数兼容的格式数据。
#        """
#        if out is None:
#            out = dict()
       
#        with self.lock:
#            timestamps = sorted(self.last_camera_data.keys())[-k:]  # 获取最近的k个时间戳
#            for i, timestamp in enumerate(timestamps):
#                if i not in out:
#                    out[i] = {
#                        'rgb': [],
#                        'timestamp': []
#                    }
#                image = self.last_camera_data[timestamp]['color']
#                out[i]['rgb'].append(image)
#                out[i]['timestamp'].append(timestamp)
       
#        for i in out:
#            out[i]['rgb'] = np.array(out[i]['rgb'])  # 转换为numpy数组，符合 (T, H, W, C) 的格式
#            out[i]['timestamp'] = np.array(out[i]['timestamp'])

#        return out

def main():
    camera_receiver = CameraReceiver(zmq_host=HOST, zmq_port=5553, topic="image_topic")

    receive_thread = threading.Thread(target=camera_receiver.receive_images)
    receive_thread.daemon = True
    receive_thread.start()

    # receive_thread = threading.Thread(target=camera_receiver.receive_images)
    # receive_thread.daemon = True
    # receive_thread.start()

    # time.sleep(2)
    # try:
    #     while True:
    #         latest_data = camera_receiver.get(k=1)
    #         assert latest_data is not None
    #         print("Latest received images:", latest_data)
    #         # 显示接收到的图像（如果需要）
    #         cv2.imshow("Received Image", latest_data[0]['rgb'][0])
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 break
    #         #time.sleep(1) 
    # except KeyboardInterrupt:
    #     print("Terminating the server.")
    # finally:
    #     print("Terminating the server.")
    #     cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
