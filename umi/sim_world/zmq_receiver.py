import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__)+'/../../')
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import zmq
import threading
import enum
import msgpack
import numpy as np
import time
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer


class Command(enum.Enum):
    SHUTDOWN = 0

class ZmqSubcriber(mp.Process):
    _instance = None
    _lock = mp.Lock()
    _status_lock = mp.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, 
            shm_manager: SharedMemoryManager,
            frequency=60,
            zmq_host='localhost', 
            zmq_port=5555,
            get_max_k=None,
            ):
        if not hasattr(self, '_initialized'):
            super().__init__(name="ReceiveInterface")            
            self.arm_joint_positions = []
            self.eef_pose = None
            self.gripper_width = 100.0
            self.cached_axis_angle = None

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
                'gripper_position': 100.0,
                'gripper_timestamp': time.time()
            }
            ring_buffer_gripper = SharedMemoryRingBuffer.create_from_examples(
                shm_manager=shm_manager,
                examples=example_gripper,
                get_max_k=get_max_k,
                get_time_budget=0.2,
                put_desired_frequency=frequency
            )

            example_vacuum = {
                'vacuum_status': 100.0,
                'vacuum_timestamp': time.time()
            }
            ring_buffer_vacuum = SharedMemoryRingBuffer.create_from_examples(
                shm_manager=shm_manager,
                examples=example_vacuum,
                get_max_k=get_max_k,
                get_time_budget=0.2,
                put_desired_frequency=frequency
            )

            example = {
                'cmd': Command.SHUTDOWN.value,
            }

            input_queue = SharedMemoryQueue.create_from_examples(
                shm_manager=shm_manager,
                examples=example,
                buffer_size=10
            )

            self.zmq_host = zmq_host
            self.zmq_port = zmq_port
            self.ready_event = mp.Event()
            self.ring_buffer_eef = ring_buffer_eef
            self.ring_buffer_gripper = ring_buffer_gripper
            self.ring_buffer_vacuum = ring_buffer_vacuum
            self.input_queue = input_queue
            self._initialized = True
            self.is_running = False

    # ========= launch method ===========
    def start(self, wait=True):
        with self._status_lock:
            if not self.is_running:
                self.is_running = True
                super().start()
                if wait:
                    self.start_wait()
                print("ros receive interface started")
            self._initialized = True
            self.is_running = False

    # ========= launch method ===========
    def start(self, wait=True):
        with self._status_lock:
            if not self.is_running:
                self.is_running = True
                super().start()
                if wait:
                    self.start_wait()
                print("ros receive interface started")
                # if self.verbose:
                #     print(f"[ReceiveInterface] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        with self._status_lock:
            if self.is_running :
                self.is_running = False
                message = {
                    'cmd': Command.SHUTDOWN.value
                }
                self.input_queue.put(message)
                print(f'zmq receiver stopped.{self.is_running}')
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

    def get_vacuum_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer_gripper.get(out=out)
        else:
            return self.ring_buffer_gripper.get_last_k(k=k, out=out)

    def get_all_state_vacuum(self):
        return self.ring_buffer_vacuum.get_all()
    
    # ========= main loop in process ============

    def run(self):   
        socket,zmq_context = self.create_socket(self.zmq_host, self.zmq_port)
        if socket is None:
            print("Failed to create socket")
            return
        exist_eef_pose=False
        exist_gripper_width=False
        exist_vacuum_status=False
        keep_running = True
        while keep_running:
            try:
                message = socket.recv_multipart(flags=zmq.NOBLOCK)
                if len(message) != 2:
                    continue

                topic, serialized_data = message
                topic = topic.decode('utf-8')
                data = msgpack.unpackb(serialized_data, raw=False)

                with open('recieve_data.txt', 'a') as f:
                    print(f"data: {topic}, {data}, recieve_time:{data['timestamp']}, latency: {int(time.time() * 1e6) - data['timestamp']}", file=f)

                if topic == "eef_pose" and isinstance(data, dict):
                    self.eef_pose = data
                    quat = [data['orientation'][k] for k in ('x', 'y', 'z', 'w')]
                    self.cached_axis_angle = self.quaternion_to_axis_angle(quat)
                    eef = dict()
                    eef['ActualTCPPose'] = [self.eef_pose['position']['x'], self.eef_pose['position']['y'], self.eef_pose['position']['z']] + self.cached_axis_angle
                    t_recv = time.time()
                    eef['robot_receive_timestamp'] = t_recv
                    eef['robot_timestamp'] = t_recv-0.03
                    self.ring_buffer_eef.put(eef)
                    if not exist_eef_pose:
                        exist_eef_pose=True
                        print('eef_pose exist')
                elif topic == "gripper_width" and isinstance(data, dict):
                    self.gripper_width = data['width']
                    gripper_states = dict()
                    t_recv = time.time()
                    gripper_states['gripper_position'] = self.gripper_width
                    gripper_states['gripper_timestamp'] = t_recv-0.03
                    self.ring_buffer_gripper.put(gripper_states)
                    if not exist_gripper_width:
                        exist_gripper_width=True
                        print('gripper_width exist')
                elif topic == "vacuum_status" and isinstance(data, dict):
                    self.vacuum_status = data['status']
                    vacuum_status = dict()
                    t_recv = time.time()
                    vacuum_status['vacuum_status'] = self.vacuum_status
                    vacuum_status['vacuum_timestamp'] = t_recv-0.03
                    self.ring_buffer_vacuum.put(vacuum_status)
                    if not exist_vacuum_status:
                        exist_vacuum_status=True
                        print('vacuum_status exist')
                if not self.ready_event.is_set() and not exist_eef_pose and not exist_gripper_width:
                    self.ready_event.set()
                    print("ready event set")
                try:
                    commands = self.input_queue.get_k(1)
                    #print(f"[sim_arm_controller] commands get from queue: {commands}")
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0
                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']
                    if cmd == Command.SHUTDOWN.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    else:
                        keep_running = False
                        break

            except zmq.Again:
                continue
            except Exception as e:
                print(f"ZMQ Error: {e}")
                break
        socket.close()
        zmq_context.term()

    def create_socket(self, zmq_host, zmq_port):
        zmq_context = zmq.Context()
        socket = zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 10000)
        socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        print("ros receive socket connected")

        for topic_filter in ["joint_states", "eef_pose", "gripper_width"]:
            socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
        return socket, zmq_context

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

if __name__ == "__main__":
    zmq_host = "localhost"
    zmq_port = 5555
    with SharedMemoryManager() as shm_manager:
        zmq_receiver = ZmqSubcriber(shm_manager, zmq_host=zmq_host, zmq_port=zmq_port)
        zmq_receiver.start()
        # try:
        #     while True:
        #         print("Actual TCP Pose:", zmq_receiver.getActualTCPPose())
        #         print("Actual Q:", zmq_receiver.getActualQ())
        #         print("Gripper Position:", zmq_receiver.getGripperCurrentPos())
        #         time.sleep(0.01)
        # except KeyboardInterrupt:
        #     pass
        # finally:
        #     print("Shutting down ZMQ interface.")