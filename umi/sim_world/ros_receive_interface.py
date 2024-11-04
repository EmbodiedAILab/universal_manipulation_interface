import zmq
import threading
import msgpack
import numpy as np
import time
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer

HOST = "192.168.1.103"

class ReceiveInterface(mp.Process):
    def __init__(self, 
            shm_manager: SharedMemoryManager,
            frequency=30,
            zmq_host=HOST, 
            zmq_port=5555,
            get_max_k=None,
            ):
        self.arm_joint_positions = []
        self.eef_pose = None
        self.gripper_width = 100.0
        self.cached_axis_angle = None
        super().__init__(name="ReceiveInterface")

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        example_eef = {
            'ActualTCPPose': np.zeros(6, dtype=np.float64),
            'robot_receive_timestamp': time.time(),
            'robot_timestamp': time.time(),
        }
        ring_buffer_eef = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example_eef,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        example_gripper = {
            'GripperCurrentPos': 0.0,
            'timestamp': time.time()
        }
        ring_buffer_gripper = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example_gripper,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.ready_event = mp.Event()
        self.ring_buffer_eef = ring_buffer_eef
        self.ring_buffer_gripper = ring_buffer_gripper

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        print("ros receive interface started")
        # if self.verbose:
        #     print(f"[ReceiveInterface] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(5)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= receive APIs =============
    # def get_state(self, k=None, out=None):
    #     if k is None:
    #         return self.ring_buffer.get(out=out)
    #     else:
    #         return self.ring_buffer.get_last_k(k=k,out=out)
    
    # def get_all_state(self):
    #     return self.ring_buffer.get_all()

    def get_eef_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer_eef.get(out=out)
        else:
            return self.ring_buffer_eef.get_last_k(k=k,out=out)
    
    def get_all_state_eef(self):
        return self.ring_buffer_eef.get_all()
    
    def get_gripper_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer_gripper.get(out=out)
        else:
            return self.ring_buffer_gripper.get_last_k(k=k,out=out)
    
    def get_all_state_gripper(self):
        return self.ring_buffer_gripper.get_all()
    
    # ========= main loop in process ============

    def run(self):
        # start connection
        # self.zmq_context = zmq.Context()
        socket = self.create_socket(self.zmq_host, self.zmq_port)
        if socket is None:
            print("Failed to create socket")
            return
        exist_eef_pose=False
        exist_gripper_width=False
        while True:
            try:
                #message = socket.recv_multipart(flags=zmq.NOBLOCK)
                message = socket.recv_multipart()
                if len(message) != 2:
                    continue

                topic, serialized_data = message
                topic = topic.decode('utf-8')
                data = msgpack.unpackb(serialized_data, raw=False)

                # if topic == "joint_states" and isinstance(data, dict):
                #     print(topic, data)
                #     self.arm_joint_positions = data['positions']
                #     # self.update_current_state()
                #     # joint_states = dict()
                #     # joint_states['ActualQ'] = self.arm_joint_positions
                #     # self.ring_buffer.put(joint_states)
                #     # print('[joint] send time: {}, receive time: {}, latency: {}'.format(
                #     #     data['timestamp'], int(time.time() * 1e6), int(time.time() * 1e6) - data['timestamp']
                #     # ))

                if topic == "eef_pose" and isinstance(data, dict):
                    self.eef_pose = data
                    quat = [data['orientation'][k] for k in ('x', 'y', 'z', 'w')]
                    self.cached_axis_angle = self.quaternion_to_axis_angle(quat)
                    eef = dict()
                    eef['ActualTCPPose'] = [self.eef_pose['position']['x'], self.eef_pose['position']['y'], self.eef_pose['position']['z']] + self.cached_axis_angle
                    t_recv = time.time()
                    eef['robot_receive_timestamp'] = t_recv
                    eef['robot_timestamp'] = data['timestamp']
                    self.ring_buffer_eef.put(eef)
                    if not exist_eef_pose:
                        exist_eef_pose=True
                        print('eef_pose exist')
                    # self.update_current_state()

                    # if self.eef_pose is None or self.cached_axis_angle is None:
                    #     continue

                    # position = self.eef_pose['position']
                    # # print('[TCPPose] send time: {}, receive time: {}, latency: {}'.format(
                    # #         self.eef_pose['timestamp'], int(time.time() * 1e6), int(time.time() * 1e6) - self.eef_pose['timestamp']
                    # #     ))
                    # pose = dict()
                    # pose['ActualTCPPose'] = [position['x'], position['y'], position['z']] + self.cached_axis_angle
                    # selumi/sim_world/ros_receive_interface.pyf.ring_buffer.put(pose)

                elif topic == "gripper_width" and isinstance(data, dict):
                    self.gripper_width = data['width']
                    gripper_states = dict()
                    gripper_states['GripperCurrentPos'] = self.gripper_width
                    gripper_states['timestamp'] = float(data['timestamp'])*1e-6
                    self.ring_buffer_gripper.put(gripper_states)
                    if not exist_gripper_width:
                        exist_gripper_width=True
                        print('gripper_width exist')
                    # self.update_current_state()
                    # print('[gripper] send time: {}, receive time: {}, latency: {}'.format(
                    #     data['timestamp'], int(time.time() * 1e6), int(time.time() * 1e6) - data['timestamp']
                    # ))
                    # gripper_states = dict()
                    # gripper_states['GripperCurrentPos'] = self.gripper_width
                    # self.ring_buffer.put(gripper_states)
                    
                # if len(self.arm_joint_positions) >= 0 and len(self.eef_pose) >= 0 and self.gripper_width >= 0:
                #     self.update_current_state()
                if not self.ready_event.is_set() and exist_eef_pose and not exist_gripper_width:
                    self.ready_event.set()
                    print("ready event set")

            except zmq.Again:
                continue
            except Exception as e:
                print(f"ZMQ Error: {e}")
                # self.zmq_context.term()
                break
        socket.close()
        # self.zmq_context.term()

    # def update_current_state(self):
    #     current_state = dict()
    #     current_state['ActualQ'] = self.arm_joint_positions
    #     current_state['ActualTCPPose'] = [self.eef_pose['position']['x'], self.eef_pose['position']['y'], self.eef_pose['position']['z']] + self.cached_axis_angle
    #     current_state['GripperCurrentPos'] = self.gripper_width
    #     self.ring_buffer.put(current_state)

    def create_socket(self, zmq_host, zmq_port):
        zmq_context = zmq.Context()
        socket = zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)  # 限制接收队列最多只保存1条消息
        socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        print("ros receive socket connected")

        for topic_filter in ["joint_states", "eef_pose", "gripper_width"]:
            socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
        return socket

    def quaternion_to_axis_angle(self, quat):
        """将四元数转换为轴角表示"""
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
    # def getActualTCPPose(self):
    #     """获取当前 TCP 位姿"""
    #     if self.eef_pose is None or self.cached_axis_angle is None:
    #         return []
    #     position = self.eef_pose['position']
    #     # print('[TCPPose] send time: {}, receive time: {}, latency: {}'.format(
    #     #         self.eef_pose['timestamp'], int(time.time() * 1e6), int(time.time() * 1e6) - self.eef_pose['timestamp']
    #     #     ))
    #     pose = dict()
    #     pose['ActualTCPPose'] = [position['x'], position['y'], position['z']] + self.cached_axis_angle
    #     self.ring_buffer.put(pose)
    #     return [position['x'], position['y'], position['z']] + self.cached_axis_angle

    # def getActualQ(self):
    #     """获取当前关节状态"""
    #     return self.arm_joint_positions if self.arm_joint_positions else []

    # # ================= Gripper Status API ===================
    # def getGripperCurrentPos(self):
    #     """获取当前夹爪宽度"""
    #     return self.gripper_width if self.gripper_width is not None else 0.0
    
    # def cleanup(self):
    #     self.socket.close()
    #     self.zmq_context.term()


if __name__ == "__main__":
    # zmq_host = "localhost"
    zmq_port = 5555
    zmq_receiver = ReceiveInterface(HOST, zmq_port)

    try:
        while True:
            print("Actual TCP Pose:", zmq_receiver.getActualTCPPose())
            print("Actual Q:", zmq_receiver.getActualQ())
            print("Gripper Position:", zmq_receiver.getGripperCurrentPos())
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down ZMQ interface.")