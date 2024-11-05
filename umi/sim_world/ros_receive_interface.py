import zmq
import threading
import pickle
import numpy as np

class ReceiveInterface:
    def __init__(self, zmq_host="localhost", zmq_port=5555):
        # 初始化 ZeroMQ 上下文和套接字
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

        self.lock = threading.Lock()
        self.arm_joint_positions = []
        self.gripper_joint_positions = []
        self.eef_pose = None
        self.gripper_width = None

        # 启动接收 ZeroMQ 消息的线程
        threading.Thread(target=self.zmq_listener, daemon=True).start()

    def zmq_listener(self):
        while True:
            try:
                # 接收消息
                topic, serialized_data = self.socket.recv_multipart()
                topic = topic.decode('utf-8')
                data = pickle.loads(serialized_data)

                # 根据 ZeroMQ 主题更新内部状态
                if topic == "joint_states_cmd" and isinstance(data, dict):
                    self.arm_joint_positions = data['positions']
                elif topic == "eef_pose_cmd" and isinstance(data, dict):
                    self.eef_pose = data
                elif topic == "gripper_width_cmd" and isinstance(data, dict):
                    self.gripper_width = data['width']
            except Exception as e:
                print(f"ZMQ Error: {e}")

    def quaternion_to_axis_angle(self, quat):
        # 将四元数转换为轴角表示
        qx, qy, qz, qw = quat
        angle = 2 * np.arccos(qw)
        sin_theta_over_two = np.sqrt(1 - qw * qw)
        if sin_theta_over_two < 1e-6:
            return [0.0, 0.0, 0.0]
        rx = qx / sin_theta_over_two
        ry = qy / sin_theta_over_two
        rz = qz / sin_theta_over_two
        return [rx * angle, ry * angle, rz * angle]
    
    
    # ================= Arm Status API ===================    
    def getActualTCPPose(self):
        if self.eef_pose is None:
            return None
        # 从 eef_pose 数据提取位置和方向
        position = self.eef_pose['position']
        rotation = self.eef_pose['orientation']
        
        # 将旋转四元数转换为轴角
        axis_angle = self.quaternion_to_axis_angle([rotation['x'], rotation['y'], rotation['z'], rotation['w']])
        
        # 返回位姿 [x, y, z, rx, ry, rz]
        return [position['x'], position['y'], position['z'], axis_angle[0], axis_angle[1], axis_angle[2]]
    
    def getActualTCPSpeed(self):
        return

    def getActualQ(self):
        with self.lock:
            return self.arm_joint_positions if self.arm_joint_positions else []
    
    def getActualQd(self):
        return
    
    def getTargetTCPPose(self):
        return  

    def getTargetTCPSpeed(self):
        return

    def getTargetQ(self):
        return

    def getTargetQd(self):
        return
    
    # ================= Gripper Status API ===================    
    def getGripperCurrentPos(self):
        with self.lock:
            return self.gripper_width if self.gripper_width is not None else 0.0

if __name__ == "__main__":
    zmq_host = "localhost"
    zmq_port = 5555
    zmq_receiver = ReceiveInterface(zmq_host, zmq_port)

    try:
        while True:
            # 打印接收到的 TCP 位置和关节信息
            print("Actual TCP Pose:", zmq_receiver.getActualTCPPose())
            print("Actual Q:", zmq_receiver.getActualQ())
            print("Gripper Position:", zmq_receiver.getGripperCurrentPos())
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down ZMQ interface.")
