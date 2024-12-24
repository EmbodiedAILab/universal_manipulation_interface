from typing import Optional, Callable, Dict,Union
import enum
import time
import cv2
import numpy as np
import multiprocessing as mp
from threadpoolctl import threadpool_limits
from multiprocessing.managers import SharedMemoryManager
from umi.common.timestamp_accumulator import get_accumulate_timestamp_idxs
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.shared_memory.shared_memory_queue import SharedMemoryQueue, Full, Empty
from umi.sim_world.camera_receive_interface import CameraReceiver
from diffusion_policy.real_world.video_recorder import VideoRecorder
from urllib.parse import urlparse

import threading

class Command(enum.Enum):
    RESTART_PUT = 0
    START_RECORDING = 1
    STOP_RECORDING = 2

class SingleCamera(mp.Process):
    """
    Call umi.common.usb_util.reset_all_elgato_devices
    if you are using Elgato capture cards.
    Required to workaround firmware bugs.
    """
    MAX_PATH_LENGTH = 4096 # linux path has a limit of 4096 bytes
    
    def __init__(
            self,
            shm_manager: SharedMemoryManager,
            # v4l2 device file path
            # e.g. /dev/video0
            # or /dev/v4l/by-id/usb-Elgato_Elgato_HD60_X_A00XB320216MTR-video-index0
            # dev_video_path,
            serial_number,
            address='localhost:5555',
            resolution=(1280, 720),
            capture_fps=30,
            put_fps=None,
            put_downsample=True,
            record_fps=None,
            get_max_k=30,
            receive_latency=0.0,
            cap_buffer_size=1,
            transform: Optional[Callable[[Dict], Dict]] = None,
            vis_transform: Optional[Callable[[Dict], Dict]] = None,
            recording_transform: Optional[Callable[[Dict], Dict]] = None,
            video_recorder: Optional[Union[VideoRecorder]] = None,
            verbose=False
        ):
        super().__init__()

        if put_fps is None:
            put_fps = capture_fps
        
        # create ring buffer
        resolution = tuple(resolution)
        shape = resolution[::-1]
        examples = {
            'color': np.empty(
                shape=shape+(3,), dtype=np.uint8)
        }
        examples['camera_capture_timestamp'] = 0.0
        examples['camera_receive_timestamp'] = 0.0
        examples['timestamp'] = 0.0
        examples['step_idx'] = 0

        vis_ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=examples if vis_transform is None 
                else vis_transform(dict(examples)),
            get_max_k=1,
            get_time_budget=0.2,
            put_desired_frequency=capture_fps
        )

        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=examples if transform is None
                else transform(dict(examples)),
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=put_fps
        )

        # create command queue
        examples = {
            'cmd': Command.RESTART_PUT.value,
            'put_start_time': 0.0,
            'video_path': np.array('a'*self.MAX_PATH_LENGTH),
            'recording_start_time': 0.0,
        }

        command_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=examples,
            buffer_size=128
        )

        # create video recorder
        if video_recorder is None:
            # realsense uses bgr24 pixel format
            # default thread_type to FRAEM
            # i.e. each frame uses one core
            # instead of all cores working on all frames.
            # this prevents CPU over-subpscription and
            # improves performance significantly
            video_recorder = VideoRecorder.create_h264(
                fps=record_fps, 
                codec='h264',
                input_pix_fmt='bgr24', 
                crf=18,
                thread_type='FRAME',
                thread_count=1)
        parsed = urlparse(address)
        host = parsed.hostname
        port = parsed.port

        self.serial_number = serial_number
        self.shm_manager = shm_manager
        self.address = address
        self.resolution = resolution
        self.capture_fps = capture_fps
        self.put_fps = put_fps
        self.put_downsample = put_downsample
        self.receive_latency = receive_latency
        self.cap_buffer_size = cap_buffer_size
        self.transform = transform
        self.vis_transform = vis_transform
        self.recording_transform = recording_transform
        self.video_recorder = video_recorder
        self.verbose = verbose
        self.put_start_time = None

        # shared variables
        self.stop_event = mp.Event()
        self.ready_event = mp.Event()
        self.ring_buffer = ring_buffer
        self.vis_ring_buffer = vis_ring_buffer
        self.command_queue = command_queue

        self.camera_receiver = CameraReceiver(shm_manager=shm_manager, zmq_host=host, zmq_port=port, topic=self.serial_number)
        self.last_frame_time = 0.0
        
    @staticmethod
    def get_connected_devices_serial():
        # 当前图像来自ros，暂时使用虚拟序列号
        serials = ["ROS2_SIMULATED_SERIAL_1"]
        return serials

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= user API ===========
    def start(self, wait=True, put_start_time=None):
        self.camera_receiver.start()
        self.put_start_time = put_start_time
        super().start()
        if wait:
            self.start_wait()
    
    def stop(self, wait=True):
        self.camera_receiver.stop()
        self.stop_event.set()
        if wait:
            self.end_wait()

    def start_wait(self):
        self.ready_event.wait()

    def end_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()

    def get(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k, out=out)
    
    def get_vis(self, out=None):
        return self.vis_ring_buffer.get(out=out)

    def start_recording(self, video_path: str, start_time: float=-1):
        path_len = len(video_path.encode('utf-8'))
        if path_len > self.MAX_PATH_LENGTH:
            raise RuntimeError('video_path too long.')
        self.command_queue.put({
            'cmd': Command.START_RECORDING.value,
            'video_path': video_path,
            'recording_start_time': start_time
        })
        
    def stop_recording(self):
        self.command_queue.put({
            'cmd': Command.STOP_RECORDING.value
        })

    def restart_put(self, start_time):
        self.command_queue.put({
            'cmd': Command.RESTART_PUT.value,
            'put_start_time': start_time
        })

    # ========= interval API ===========
    def run(self):
        put_idx = None
        put_start_time = self.put_start_time
        if put_start_time is None:
            put_start_time = time.time()

        # reuse frame buffer
        iter_idx = 0
        t_start = time.time()

        while not self.stop_event.is_set():
            ts = time.time()
            frame_data = self.camera_receiver.get_image(k=1)
            if frame_data == {} or frame_data[0]['timestamp'][0] == self.last_frame_time:
                continue
            self.last_frame_time = frame_data[0]['timestamp'][0]

            # print("single camera, received images:", frame_data)
            assert frame_data is not None
            
            frame = frame_data[0]['rgb'][0]
            t_recv = time.time()
            t_cap = frame_data[0]['timestamp'][0] / 1000000.0  # assuming timestamp is in milliseconds
            t_cal = t_recv - self.receive_latency  # calibrated latency
            # print('frame: ', frame_data[0]['rgb'][0])
            # print('==== t_recv 接收时时间：', t_recv)
            # print('==== t_cap 捕获时时间计算前：', frame_data[0]['timestamp'][0])
            # print('==== t_cap 捕获时时间：', t_cap)
            # print('==== t_cal 校准后时间：', t_cal)

            data = dict()
            data['camera_receive_timestamp'] = t_recv
            data['camera_capture_timestamp'] = t_cap
            data['color'] = frame
            
            # apply transform
            put_data = data
            if self.transform is not None:
                put_data = self.transform(dict(data))

            if self.put_downsample:                
                # put frequency regulation
                local_idxs, global_idxs, put_idx \
                    = get_accumulate_timestamp_idxs(
                        timestamps=[t_cal],
                        start_time=put_start_time,
                        dt=1/self.put_fps,
                        # this is non in first iteration
                        # and then replaced with a concrete number
                        next_global_idx=put_idx,
                        # continue to pump frames even if not started.
                        # start_time is simply used to align timestamps.
                        allow_negative=True
                    )

                for step_idx in global_idxs:
                    put_data['step_idx'] = step_idx
                    put_data['timestamp'] = t_cal
                    self.ring_buffer.put(put_data, wait=True)
            else:
                step_idx = int((t_cal - put_start_time) * self.put_fps)
                put_data['step_idx'] = step_idx
                put_data['timestamp'] = t_cal
                self.ring_buffer.put(put_data, wait=True)
            # print('[single camera] put_data: ', put_data)

            # signal ready
            if iter_idx == 0 and put_data != {}:
                self.ready_event.set()
                
            # put to vis
            vis_data = data
            if self.vis_transform == self.transform:
                vis_data = put_data
            elif self.vis_transform is not None:
                vis_data = self.vis_transform(dict(data))
            self.vis_ring_buffer.put(vis_data, wait=True)

            # record frame
            rec_data = data
            if self.recording_transform == self.transform:
                rec_data = put_data
            elif self.recording_transform is not None:
                rec_data = self.recording_transform(dict(data))

            if self.video_recorder.is_ready():
                self.video_recorder.write_frame(rec_data['color'],
                                                frame_time=t_cal)

            # perf
            t_end = time.time()
            duration = t_end - t_start
            frequency = np.round(1 / duration, 1)
            t_start = t_end
            if self.verbose:
                print(f'[UvcCamera] FPS {frequency}')


            # fetch command from queue
            try:
                commands = self.command_queue.get_all()
                n_cmd = len(commands['cmd'])
            except Empty:
                n_cmd = 0

            # execute commands
            for i in range(n_cmd):
                command = dict()
                for key, value in commands.items():
                    command[key] = value[i]
                cmd = command['cmd']
                if cmd == Command.RESTART_PUT.value:
                    put_idx = None
                    put_start_time = command['put_start_time']
                elif cmd == Command.START_RECORDING.value:
                    video_path = str(command['video_path'])
                    start_time = command['recording_start_time']
                    if start_time < 0:
                        start_time = None
                    self.video_recorder.start(video_path, start_time=start_time)
                elif cmd == Command.STOP_RECORDING.value:
                    self.video_recorder.stop()
                    # stop need to flush all in-flight frames to disk, which might take longer than dt.
                    # soft-reset put to drop frames to prevent ring buffer overflow.
                    put_idx = None
                elif cmd == Command.RESTART_PUT.value:
                    put_idx = None
                    put_start_time = command['put_start_time']

            iter_idx += 1

