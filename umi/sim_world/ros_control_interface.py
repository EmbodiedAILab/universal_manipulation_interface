import zmq
import time
# import pickle
import msgpack
from scipy.spatial.transform import Rotation as R

class ControlInterface:
    def __init__(self, zmq_host='localhost', zmq_port=5554):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 1000)

        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")
        self.send_count = 0

    def send_message(self, topic, data):
        """通过 ZeroMQ 发布序列化的消息"""     
        try: 
            serialized_data = msgpack.packb(data, use_bin_type=True)
            self.socket.send_multipart([topic.encode(), serialized_data])
            # send_time = time.time()
            # with open("pose_data.txt", 'a') as f:
            #     print(f"pose data: {data}, send time: {send_time}", file=f)
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

        rotation_vector = [pose[3], pose[4], pose[5]]

        r = R.from_rotvec(rotation_vector)
        quaternion = r.as_quat()

        pose_data = {
            'position': {'x': pose[0], 'y': pose[1], 'z': pose[2]},
            'orientation': {'x': quaternion[0], 'y': quaternion[1], 'z': quaternion[2], 'w': quaternion[3]},
            'timestamp': int(time.time() * 1e6),
        }
        self.send_count += 1

        return self.send_message('servoL_cmd', pose_data)

    def moveJ(self, joint_positions):
        if not isinstance(joint_positions, list) or not all(isinstance(pos, (int, float)) for pos in joint_positions):
            print("Joint positions must be a list of numbers")
            return

        joint_data = {
            'positions': joint_positions,
            'timestamp': int(time.time() * 1e6)
        }

        return self.send_message('moveJ_cmd', joint_data)
    
    def cleanup(self):
            self.socket.close()
            self.context.term()


class GripperControlInterface:
    def __init__(self, zmq_host='localhost', zmq_port=5556):
        self.context = zmq.Context(io_threads=2)
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 1000)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

    def send_message(self, topic, data):
        """通过 ZeroMQ 发布序列化的消息"""     
        try: 
            serialized_data = msgpack.packb(data, use_bin_type=True)
            self.socket.send_multipart([topic.encode(), serialized_data])
            # print(f"sent gripper_cmd: {data}")
            return True
        except zmq.ZMQError as e:
            print(f"send failed with error: {e}")
            return False

    def setGripperTargetPos(self, target_pos):
        gripper_data = {
            'positions': target_pos,
            'timestamp': int(time.time() * 1e6)
        }

        return self.send_message('gripper_cmd', gripper_data)
    
    def cleanup(self):
            print('cleanup')
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.close()
            self.context.term()


if __name__ == "__main__":
    zmq_control = ControlInterface()
    gripper_control = GripperControlInterface()

    example_pose = [1.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    example_joint_positions = [0.5, 1.2, -0.8, 0.0, 0.3, -1.1]
    gripper_position = 100

    try:
        while True:
            zmq_control.servoL(example_pose)
            gripper_control.setGripperTargetPos(gripper_position)
            # zmq_control.moveJ(example_joint_positions)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("Stopping the control interface.")
    finally:
        zmq_control.cleanup()
        gripper_control.cleanup()
        print("ZeroMQ connection cleaned up.")