import zmq
import threading
import pickle
import numpy as np
import time

class ReceiveInterface:
    def __init__(self, zmq_host="localhost", zmq_port=5555):
        self.zmq_context = zmq.Context()
        self.sockets = {
            "joint_states": self.create_socket(zmq_host, zmq_port, "joint_states"),
            "eef_pose": self.create_socket(zmq_host, zmq_port, "eef_pose"),
            "gripper_width": self.create_socket(zmq_host, zmq_port, "gripper_width")
        }

        self.locks = {
            "joint_states": threading.Lock(),
            "eef_pose": threading.Lock(),
            "gripper_width": threading.Lock()
        }
        self.arm_joint_positions = []
        self.eef_pose = None
        self.gripper_width = None

        threading.Thread(target=self.zmq_listener, args=("joint_states",), daemon=True).start()
        threading.Thread(target=self.zmq_listener, args=("eef_pose",), daemon=True).start()
        threading.Thread(target=self.zmq_listener, args=("gripper_width",), daemon=True).start()

    def create_socket(self, zmq_host, zmq_port, topic_filter):
        socket = self.zmq_context.socket(zmq.SUB)
        socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
        return socket

    def zmq_listener(self, topic_filter):
        socket = self.sockets[topic_filter]

        while True:
            try:
                message = socket.recv_multipart()
                if len(message) != 2:
                    continue
                topic, serialized_data = message
                topic = topic.decode('utf-8')
                data = pickle.loads(serialized_data)
                # print(f"========= topic: {topic}, data: {data}")

                with self.locks[topic_filter]:
                    if topic == "joint_states" and isinstance(data, dict):
                        self.arm_joint_positions = data['positions']
                    elif topic == "eef_pose" and isinstance(data, dict):
                        self.eef_pose = data
                        # print(f"eef_pose: {data}")
                    elif topic == "gripper_width" and isinstance(data, dict):
                        self.gripper_width = data['width']
                        # print(f"gripper_width: {self.gripper_width}")
            except Exception as e:
                print(f"ZMQ Error ({topic_filter}): {e}")

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
        with self.locks["eef_pose"]:
            if self.eef_pose is None:
                return []
            position = self.eef_pose['position']
            rotation = self.eef_pose['orientation']
            
        axis_angle = self.quaternion_to_axis_angle([rotation['x'], rotation['y'], rotation['z'], rotation['w']])
            
        return [position['x'], position['y'], position['z'], axis_angle[0], axis_angle[1], axis_angle[2]]
    
    def getActualQ(self):
        with self.locks["joint_states"]:
            return self.arm_joint_positions if self.arm_joint_positions else []
    
    # ================= Gripper Status API ===================
    def getGripperCurrentPos(self):
        with self.locks["gripper_width"]:
            return self.gripper_width if self.gripper_width is not None else 0.0    

if __name__ == "__main__":
    zmq_host = "localhost"
    zmq_port = 5555
    zmq_receiver = ReceiveInterface(zmq_host, zmq_port)

    try:
        while True:
            print("Actual TCP Pose:", zmq_receiver.getActualTCPPose())
            # print("Actual Q:", zmq_receiver.getActualQ())
            # zmq_receiver.getGripperCurrentPos()
            # print("Gripper Position:", zmq_receiver.getGripperCurrentPos())
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down ZMQ interface.")
