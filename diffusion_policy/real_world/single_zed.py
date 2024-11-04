from typing import Optional, Callable, Dict
import os
import enum
import time
import json
import numpy as np
import pyzed.sl as sl
import multiprocessing as mp
import cv2
from threadpoolctl import threadpool_limits
from multiprocessing.managers import SharedMemoryManager
from diffusion_policy.common.timestamp_accumulator import get_accumulate_timestamp_idxs
from diffusion_policy.shared_memory.shared_ndarray import SharedNDArray
from diffusion_policy.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from diffusion_policy.shared_memory.shared_memory_queue import SharedMemoryQueue, Full, Empty
from diffusion_policy.real_world.video_recorder import VideoRecorder
from diffusion_policy.real_world.gstreamer_recorder import GStreamerRecorder


class Command(enum.Enum):
    SET_COLOR_OPTION = 0
    SET_DEPTH_OPTION = 1
    START_RECORDING = 2
    STOP_RECORDING = 3
    RESTART_PUT = 4

class SingleZed(mp.Process):
    MAX_PATH_LENGTH = 4096 # linux path has a limit of 4096 bytes

    def __init__(
            self, 
            shm_manager: SharedMemoryManager,
            serial_number,
            resolution=(1280,720),
            capture_fps=30,
            put_fps=None,
            put_downsample=True,
            record_fps=None,
            enable_color=True,
            enable_depth=False,
            enable_infrared=False,
            get_max_k=30,
            advanced_mode_config=None,
            transform: Optional[Callable[[Dict], Dict]] = None,
            vis_transform: Optional[Callable[[Dict], Dict]] = None,
            recording_transform: Optional[Callable[[Dict], Dict]] = None,
            video_recorder: Optional[VideoRecorder] = None,
            verbose=False
        ):
        super().__init__()

        if put_fps is None:
            put_fps = capture_fps
        if record_fps is None:
            record_fps = capture_fps

        # create ring buffer
        resolution = tuple(resolution)
        shape = resolution[::-1]
        examples = dict()
        if enable_color:
            examples['color'] = np.empty(
                shape=shape+(3,), dtype=np.uint8)
        if enable_depth:
            examples['depth'] = np.empty(
                shape=shape, dtype=np.uint16)
        if enable_infrared:
            examples['infrared'] = np.empty(
                shape=shape, dtype=np.uint8)
        
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
            'cmd': Command.SET_COLOR_OPTION.value,
            'option_value': 0.0,
            'video_path': np.array('a'*self.MAX_PATH_LENGTH),
            'recording_start_time': 0.0,
            'put_start_time': 0.0
        }

        command_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=examples,
            buffer_size=128
        )

        # create shared array for intrinsics
        intrinsics_array = SharedNDArray.create_from_shape(
                mem_mgr=shm_manager,
                shape=(7,),
                dtype=np.float64)
        intrinsics_array.get()[:] = 0

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

        # copied variables
        print("construct sl camera")
        self.cam = sl.Camera()
        cam_param = sl.InitParameters()
        cam_param.camera_resolution = sl.RESOLUTION.AUTO
        cam_param.camera_fps = 30
        print("construct sl camera done")
        self.cam_param = cam_param
        self.resolution = resolution
        self.capture_fps = capture_fps
        self.put_fps = put_fps
        self.put_downsample = put_downsample
        self.record_fps = record_fps
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.enable_infrared = enable_infrared
        self.advanced_mode_config = advanced_mode_config
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
        self.intrinsics_array = intrinsics_array
    
    @staticmethod
    def get_connected_devices_serial():
        serials = list()
        for d in rs.context().devices:
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                serial = d.get_info(rs.camera_info.serial_number)
                product_line = d.get_info(rs.camera_info.product_line)
                if product_line == 'D400':
                    # only works with D400 series
                    serials.append(serial)
        serials = sorted(serials)
        return serials

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= user API ===========
    def start(self, wait=True, put_start_time=None):
        self.put_start_time = put_start_time
        super().start()
        if wait:
            self.start_wait()
    
    def stop(self, wait=True):
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
    
    # ========= user API ===========
    # def set_color_option(self, option: rs.option, value: float):
    #     self.command_queue.put({
    #         'cmd': Command.SET_COLOR_OPTION.value,
    #         'option_enum': option.value,
    #         'option_value': value
    #     })
    
    # def set_exposure(self, exposure=None, gain=None):
    #     """
    #     exposure: (1, 10000) 100us unit. (0.1 ms, 1/10000s)
    #     gain: (0, 128)
    #     """

    #     if exposure is None and gain is None:
    #         # auto exposure
    #         self.set_color_option(rs.option.enable_auto_exposure, 1.0)
    #     else:
    #         # manual exposure
    #         self.set_color_option(rs.option.enable_auto_exposure, 0.0)
    #         if exposure is not None:
    #             self.set_color_option(rs.option.exposure, exposure)
    #         if gain is not None:
    #             self.set_color_option(rs.option.gain, gain)
    
    # def set_white_balance(self, white_balance=None):
    #     if white_balance is None:
    #         self.set_color_option(rs.option.enable_auto_white_balance, 1.0)
    #     else:
    #         self.set_color_option(rs.option.enable_auto_white_balance, 0.0)
    #         self.set_color_option(rs.option.white_balance, white_balance)

    # def get_intrinsics(self):
    #     assert self.ready_event.is_set()
    #     fx, fy, ppx, ppy = self.intrinsics_array.get()[:4]
    #     mat = np.eye(3)
    #     mat[0,0] = fx
    #     mat[1,1] = fy
    #     mat[0,2] = ppx
    #     mat[1,2] = ppy
    #     return mat

    # def get_depth_scale(self):
    #     assert self.ready_event.is_set()
    #     scale = self.intrinsics_array.get()[-1]
    #     return scale
    
    def start_recording(self, video_path: str, start_time: float=-1):
        assert self.enable_color

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
        # limit threads
        threadpool_limits(1)
        cv2.setNumThreads(1)

        w, h = self.resolution
        fps = self.capture_fps
        
        try:
            # start pipeline
            print("open sl camera")
            err = self.cam.open(self.cam_param)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"Camera Open: {repr(err)}")
            print("open sl camera done")
            runtime_parameters = sl.RuntimeParameters()
            
            # put frequency regulation
            put_idx = None
            put_start_time = self.put_start_time
            if put_start_time is None:
                put_start_time = time.time()

            iter_idx = 0
            t_start = time.time()
            image_resolution = self.cam.get_camera_information().camera_configuration.resolution
            print("construct sl mat", image_resolution)
            image = sl.Mat()
            print("construct sl mat done")
            while not self.stop_event.is_set():
                data = dict()
                # wait for frames to come in
                if self.cam.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                    self.cam.retrieve_image(image, sl.VIEW.LEFT)
                    receive_time = time.time()
                    # data['camera_capture_timestamp'] = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_milliseconds()
                    data['camera_receive_timestamp'] = receive_time
                    # print("Image resolution: {0} x {1}\n".format(image.get_width(), image.get_height()))
                    data['color'] = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
                    # print("image shape: ", data['color'].shape)
                    #cv2.imshow("image", data['color'])
                    #key = cv2.waitKey(10)
                    # continue
                    # apply transform
                    put_data = data
                    if self.transform is not None:
                        put_data = self.transform(dict(data))

                    if self.put_downsample:                
                        # put frequency regulation
                        local_idxs, global_idxs, put_idx \
                            = get_accumulate_timestamp_idxs(
                                timestamps=[receive_time],
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
                            # put_data['timestamp'] = put_start_time + step_idx / self.put_fps
                            put_data['timestamp'] = receive_time
                            # print(step_idx, data['timestamp'])
                            self.ring_buffer.put(put_data, wait=False)
                    else:
                        step_idx = int((receive_time - put_start_time) * self.put_fps)
                        put_data['step_idx'] = step_idx
                        put_data['timestamp'] = receive_time
                        self.ring_buffer.put(put_data, wait=False)

                    # signal ready
                    if iter_idx == 0:
                        self.ready_event.set()
                
                    # put to vis
                    vis_data = data
                    if self.vis_transform == self.transform:
                        vis_data = put_data
                    elif self.vis_transform is not None:
                        vis_data = self.vis_transform(dict(data))
                    self.vis_ring_buffer.put(vis_data, wait=False)

                    #print('record')
                    # record frame
                    rec_data = data
                    if self.recording_transform == self.transform:
                        rec_data = put_data
                    elif self.recording_transform is not None:
                        rec_data = self.recording_transform(dict(data))

                    if self.video_recorder.is_ready():
                        self.video_recorder.write_frame(rec_data['color'], 
                            frame_time=receive_time)

                # perf
                t_end = time.time()
                duration = t_end - t_start
                frequency = np.round(1 / duration, 1)
                t_start = t_end
                if self.verbose:
                    print(f'[SingleZed 0000] FPS {frequency}')

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
                    if cmd == Command.START_RECORDING.value:
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
                        # self.ring_buffer.clear()

                iter_idx += 1
        except Exception as ee:
            print(ee)
        finally:
            self.video_recorder.stop()
            self.cam.close()
            self.ready_event.set()
        
        if self.verbose:
            print(f'[SingleRealsense 0000] Exiting worker process.')
