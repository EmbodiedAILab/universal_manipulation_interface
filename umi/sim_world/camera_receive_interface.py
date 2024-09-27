import socket
import os
import struct
import time
import cv2
import numpy as np
from struct import unpack

class CameraReceiver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.last_camera_data = {}  # 用来保存接收到的图像数据

    def receive_images(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        print(f"Server listening on {self.host}:{self.port}")

        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection from {addr}")

            try:
                while True:
                    header = client_socket.recv(16)
                    if not header or len(header) != 16:
                        print("No more data or header incomplete. Closing connection.")
                        break

                    timestamp, img_size = unpack('!QQ', header)
                    img_data = b''
                    while len(img_data) < img_size:
                        packet = client_socket.recv(min(4096, img_size - len(img_data)))
                        if not packet:
                            break
                        img_data += packet

                    if len(img_data) != img_size:
                        print("Image data incomplete.")
                        continue

                    image = cv2.imdecode(np.asarray(bytearray(img_data), dtype='uint8'), cv2.IMREAD_COLOR)

                    if image is not None:
                        self.last_camera_data[timestamp] = {
                            'color': image,
                            'timestamp': timestamp
                        }
                        cv2.imshow("MetaEngine", image)
                        cv2.pollKey()

            except Exception as e:
                print(f"An error occurred while receiving data: {e}")
            finally:
                client_socket.close()
                print(f"Connection closed with {addr}")

    def get(self, k=None, out=None) -> dict:
        """
        返回与 'camera.get()' 函数兼容的格式数据
        """
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

        return out

if __name__ == "__main__":
    host = "0.0.0.0"
    port = 65433

    camera_receiver = CameraReceiver(host, port)
    camera_receiver.receive_images()
    camera_receiver.get()
