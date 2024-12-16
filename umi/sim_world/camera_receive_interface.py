import zmq
import cv2
import time
import enum
import numpy as np
import pickle
import threading
from collections import deque
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer

class Command(enum.Enum):
    SHUTDOWN = 0

class CameraReceiver(mp.Process):
    def __init__(self, 
            shm_manager: SharedMemoryManager,
            frequency=30,
            zmq_host="127.0.0.1", 
            zmq_port=5555, 
            topic="image_topic", 
            max_cache=300,
            get_max_k=None,
            resolution=(1280, 720),
            ):

        super().__init__(name="CameraReceiver")
        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.topic = topic
        self.max_cache = max_cache
        self.resolution = resolution
        self.image_shape = (self.resolution[1], self.resolution[0], 3)

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        example = {
            'color': np.zeros(self.image_shape, dtype=np.uint8),
            'timestamp': time.time(),
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        example_vis = {
            'color': np.zeros(self.image_shape, dtype=np.uint8),
            'timestamp': time.time(),
        }
        vis_ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example_vis,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        example_cmd = {
            'cmd': Command.SHUTDOWN.value,
        }

        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example_cmd,
            buffer_size=10
        )

        self.ready_event = mp.Event()
        self.ring_buffer = ring_buffer
        self.vis_ring_buffer = vis_ring_buffer
        self.input_queue = input_queue
        self.is_running = False

    # ========= launch method ===========
    def start(self, wait=True):
        if not self.is_running:
            self.is_running = True
            super().start()
            if wait:
                self.start_wait()
            print("camera interface started")

    def stop(self, wait=True):
        if self.is_running :
            self.is_running = False
            message = {
                'cmd': Command.SHUTDOWN.value
            }
            self.input_queue.put(message)
            print(f'camera interface stopped.{self.is_running}')
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
    def get_image(self, k=None, out=None):
        if k is None:
            image = self.ring_buffer.get(out=out)
            return self.to_image_format(image)
        else:
            image = self.ring_buffer.get_last_k(k=k,out=out)
            return self.to_image_format(image)
    
    def get_all_image(self):
        return self.ring_buffer.get_all()

    # ========= main loop in process ============
    def run(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)

        exist_image_frame = False
        keep_running = True
        while keep_running:
            try:
                message = self.socket.recv()
                _, serialized_data = message.split(b" ", 1)
                receive_time = time.time()

                cv_image = pickle.loads(serialized_data)

                # with self.lock:
                #     self.last_camera_data.append({
                #         'color': cv_image,
                #         'timestamp': int(receive_time * 1e6)
                #     })
                # cv2.imshow("Received Image", cv_image)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
                t_recv = time.time()
                frame = dict()
                frame['color'] = cv_image
                frame['timestamp'] = t_recv
                self.ring_buffer.put(frame)

                if not exist_image_frame:
                    exist_image_frame = True
                    print('image exist')

                # event ready
                if not self.ready_event.is_set() and not exist_image_frame:
                    self.ready_event.set()
                    print("ready event set")

                # command handle
                try:
                    commands = self.input_queue.get_k(1)
                    #print(f"[camera_receive] commands get from queue: {commands}")
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

            except Exception as e:
                print(f"An error occurred while receiving image: {e}")
                break
        self.socket.close()
        self.context.term()

    def to_image_format(self, recent_data) -> dict:
        out = {}
        num_entries = recent_data['color'].shape[0]

        for i in range(num_entries):
            if i not in out:
                out[i] = {
                    'rgb': [],
                    'timestamp': []
                }
            out[i]['rgb'].append(recent_data['color'][i])
            out[i]['timestamp'].append(recent_data['timestamp'][i])

        for i in out:
            out[i]['rgb'] = np.array(out[i]['rgb'])  # (T, H, W, C)
            out[i]['timestamp'] = np.array(out[i]['timestamp'])  # (T,)

        return out


def main():
    camera_receiver = CameraReceiver(zmq_host="localhost", zmq_port=5555, topic="image_topic")

    receive_thread = threading.Thread(target=camera_receiver.receive_images)
    receive_thread.daemon = True
    receive_thread.start()

    try:
        while True:
            latest_data = camera_receiver.get(k=1)
            if latest_data != {}:
                print('latest data: ', latest_data)
                cv_image = latest_data[0]['rgb'][0]
                cv2.imshow("Received Image", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            print("Latest received images:", latest_data)
            # time.sleep(1)
    except KeyboardInterrupt:
        print("Terminating the server.")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()