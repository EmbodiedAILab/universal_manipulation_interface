import zmq
import time
import pickle
from scipy.spatial.transform import Rotation as R

class ControlInterface:
    def __init__(self, zmq_host="localhost", zmq_port=5554):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

    def send_message(self, topic, data):
        """通过 ZeroMQ 发布序列化的消息"""     
        try: 
            serialized_data = pickle.dumps(data)
            self.socket.send_multipart([topic.encode(), serialized_data])
            return True
        except zmq.ZMQError as e:
            print(f"send failed with error: {e}")
            return False

    def servoL(self, pose):
        """
        接受位姿数据 [x, y, z, rx, ry, rz]（旋转部分为轴角表示）并通过 ZeroMQ 发布到 'servoL_cmd' 主题。
        :param pose: 位姿数据 [x, y, z, rx, ry, rz]，其中 [rx, ry, rz] 是旋转轴角表示。
        """
        if len(pose) != 6:
            print("Pose must have 6 elements: [x, y, z, rx, ry, rz]")
            return

        # 将 [rx, ry, rz] 作为旋转向量（轴角表示）
        rotation_vector = [pose[3], pose[4], pose[5]]  # 旋转向量，表示旋转轴和旋转角度

        # 创建旋转对象，从轴角表示（旋转向量）生成四元数
        r = R.from_rotvec(rotation_vector)  # 从旋转向量生成旋转对象
        quaternion = r.as_quat()  # 获取四元数 [x, y, z, w]

        # 准备消息数据
        pose_data = {
            'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]},
            'orientation': {'x': quaternion[0], 'y': quaternion[1], 'z': quaternion[2], 'w': quaternion[3]},
            'timestamp': int(time.time() * 1e6)
        }

        # 通过 ZeroMQ 发布消息
        return self.send_message('servoL_cmd', pose_data)

    def moveJ(self, joint_positions):
        if not isinstance(joint_positions, list) or not all(isinstance(pos, (int, float)) for pos in joint_positions):
            print("Joint positions must be a list of numbers")
            return

        # 准备消息数据
        joint_data = {
            'positions': joint_positions,
            'timestamp': int(time.time() * 1e6)
        }

        # 通过 ZeroMQ 发布消息
        return self.send_message('moveJ_cmd', joint_data)

    def setGripperTargetPos(self, target_pos):
        joint_positions = [-target_pos / 2, -target_pos / 2]
        
        # 准备消息数据
        gripper_data = {
            'positions': joint_positions,
            'timestamp': int(time.time() * 1e6)
        }

        # 通过 ZeroMQ 发布消息
        return self.send_message('gripper_cmd', gripper_data)


# 示例使用
if __name__ == "__main__":
    zmq_control = ControlInterface()

    # 示例数据
    example_pose = [1.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    example_joint_positions = [0.5, 1.2, -0.8, 0.0, 0.3, -1.1]
    gripper_position = 0.04  # 示例夹爪目标位置

    # 发送示例消息
    while True:
        zmq_control.servoL(example_pose)
        # zmq_control.moveJ(example_joint_positions)
        # zmq_control.setGripperTargetPos(gripper_position)
