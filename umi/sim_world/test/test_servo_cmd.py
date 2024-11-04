import zmq
import pickle

if __name__ == "__main__":
    zmq_host = "localhost"
    zmq_port = 5555
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://{zmq_host}:{zmq_port}")

    # 在此处加入触发发送的逻辑，例如通过按键或定时器
    while True:
        # 示例触发逻辑
        pose_data = {
            'position': {'x': 1.0, 'y': 0.0, 'z': 0.5},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        serialized_data = pickle.dumps(pose_data)
        socket.send_multipart([b"servoL_cmd", serialized_data])
        print(f"Sent data: {pose_data}")
