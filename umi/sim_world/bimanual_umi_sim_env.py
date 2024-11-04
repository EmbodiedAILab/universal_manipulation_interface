from typing import Optional, List
import pathlib
import numpy as np
import time
import shutil
import math
from multiprocessing.managers import SharedMemoryManager

# from umi.sim_world.single_camera import SingleCamera
# from umi.sim_world.multi_camera import MultiCameras

from diffusion_policy.real_world.multi_camera import MultiCamera as MultiCamera
from diffusion_policy.real_world.single_realsense import SingleRealsense as SingleCamera
# from diffusion_policy.real_world.multi_realsense import MultiRealsense as MultiCamera
# from diffusion_policy.real_world.single_realsense import SingleRealsense as SingleCamera
from diffusion_policy.real_world.gstreamer_recorder import GStreamerRecorder
from diffusion_policy.real_world.video_recorder import VideoRecorder

# from umi.real_world.rtde_interpolation_controller import RTDEInterpolationController
# from umi.real_world.dh_controller import DHController

from umi.sim_world.sim_arm_controller import SimArmController
from umi.sim_world.sim_gripper_controller import SimGripperController
# from umi.sim_world.ros_control_interface import ControlInterface

# from umi.real_world.rtde_interpolation_controller import RTDEInterpolationController
# from umi.real_world.wsg_controller import WSGController
# from umi.real_world.dh_controller import DHController
# from umi.real_world.franka_interpolation_controller import FrankaInterpolationController
# from umi.real_world.multi_uvc_camera import MultiUvcCamera
# =============================
from diffusion_policy.common.timestamp_accumulator import (
    TimestampActionAccumulator,
    ObsAccumulator
)
# from umi.common.cv_util import (draw_predefined_mask, draw_gripper_mask)
from umi.real_world.multi_camera_visualizer import MultiCameraVisualizer
from diffusion_policy.common.replay_buffer import ReplayBuffer
from diffusion_policy.common.cv2_util import (
    get_image_transform, optimal_row_cols)
from umi.common.cv_util import (
    get_image_transform2
)
from umi.common.usb_util import reset_all_elgato_devices, get_sorted_v4l_paths
from umi.common.pose_util import pose_to_pos_rot
from umi.common.interpolation_util import get_interp1d, PoseInterpolator


class BimanualUmiSimEnv:
    def __init__(self, 
            # required params
            output_dir,
            robots_config, # list of dict[{robot_type: 'ur5', robot_ip: XXX, obs_latency: 0.0001, action_latency: 0.1, tcp_offset: 0.21}]
            grippers_config, # list of dict[{gripper_ip: XXX, gripper_port: 1000, obs_latency: 0.01, , action_latency: 0.1}]
            # env params
            frequency=20,
            # obs
            obs_image_resolution=(640,480), #确认是否改为720
            max_obs_buffer_size=60,
            obs_float32=False,
            camera_reorder=None,
            no_mirror=False,
            fisheye_converter=None,
            mirror_swap=False,
            # this latency compensates receive_timestamp
            # all in seconds
            camera_obs_latency=0.125,
            # all in steps (relative to frequency)
            camera_down_sample_steps=1,
            robot_down_sample_steps=1,
            gripper_down_sample_steps=1,
            # all in steps (relative to frequency)
            camera_obs_horizon=2,
            robot_obs_horizon=2,
            gripper_obs_horizon=2,
            # action
            max_pos_speed=0.25,
            max_rot_speed=0.6,
            init_joints=False,
            # vis params
            enable_multi_cam_vis=True,
            multi_cam_vis_resolution=(1280, 720),
            # shared memory
            shm_manager=None
            ):
        output_dir = pathlib.Path(output_dir)
        assert output_dir.parent.is_dir()
        video_dir = output_dir.joinpath('videos')
        video_dir.mkdir(parents=True, exist_ok=True)
        zarr_path = str(output_dir.joinpath('replay_buffer.zarr').absolute())
        replay_buffer = ReplayBuffer.create_from_path(
            zarr_path=zarr_path, mode='a')

        if shm_manager is None:
            shm_manager = SharedMemoryManager()
            shm_manager.start()

        camera_serial_numbers = SingleCamera.get_connected_devices_serial()
        print(f'camera_serial_numbers: {camera_serial_numbers}')
        video_capture_resolution = (1280,720)
        capture_fps = 30

        # video_recorder = list()
        transform = list()
        vis_transform = list()
        color_tf = get_image_transform(
            input_res=video_capture_resolution,
            output_res=obs_image_resolution,
            # obs output rgb
            bgr_to_rgb=True)
        color_transform = color_tf
        if obs_float32:
            color_transform = lambda x: color_tf(x).astype(np.float32) / 255

        def transform(data):
            data['color'] = color_transform(data['color'])
            return data

        rw, rh, col, row = optimal_row_cols(
            n_cameras=len(camera_serial_numbers)+1,
            in_wh_ratio=obs_image_resolution[0] / obs_image_resolution[1],
            max_resolution=multi_cam_vis_resolution
        )
        vis_color_transform = get_image_transform(
            input_res=video_capture_resolution,
            output_res=(rw, rh),
            bgr_to_rgb=False
        )

        def vis_transform(data):
            data['color'] = vis_color_transform(data['color'])
            return data

        recording_transfrom = None
        recording_fps = capture_fps
        recording_pix_fmt = 'bgr24'
        # if not record_raw_video:
        #     recording_transfrom = transform
        #     recording_fps = frequency
        #     recording_pix_fmt = 'rgb24'
        
        # 数据录制
        video_recorder = VideoRecorder.create_h264(
            fps=recording_fps,
            codec='h264',
            input_pix_fmt=recording_pix_fmt,
            crf=21,
            thread_type='FRAME',
            thread_count=2)

        camera = MultiCamera(
            serial_numbers=['317422072109','zed','234222301774'],
            shm_manager=shm_manager,
            resolution=video_capture_resolution,
            capture_fps=capture_fps,
            # send every frame immediately after arrival
            # ignores put_fps
            put_downsample=False,
            record_fps=capture_fps,
            enable_color=True,
            enable_depth=False,
            enable_infrared=False,
            get_max_k=max_obs_buffer_size,
            transform=transform,
            vis_transform=vis_transform,
            recording_transform=recording_transfrom,
            video_recorder=video_recorder,
            verbose=False
        )

        multi_cam_vis = None
        if enable_multi_cam_vis:
            multi_cam_vis = MultiCameraVisualizer(
                camera=camera,
                row=row,
                col=col,
                vis_fps=30,
                rgb_to_bgr=False
            )

        cube_diag = np.linalg.norm([1,1,1])
        j_init = np.array([0,-90,-90,-90,90,0]) / 180 * np.pi
        if not init_joints:
            j_init = None

        assert len(robots_config) == len(grippers_config)
        # robots: List[RTDEInterpolationController] = list()
        # grippers: List[DHController] = list()
        robots: List[SimArmController] = list()
        grippers: List[SimGripperController] = list()
        for rc in robots_config:
            if rc['robot_type'].startswith('ur5'):
                assert rc['robot_type'] in ['ur5', 'ur5e']
                # this_robot = RTDEInterpolationController(
                this_robot = SimArmController(
                    shm_manager=shm_manager,
                    robot_ip=rc['robot_ip'],
                    frequency=50, #if rc['robot_type'] == 'ur5e' else 125,
                    lookahead_time=0.1,
                    gain=300,
                    max_pos_speed=max_pos_speed*cube_diag,
                    max_rot_speed=max_rot_speed*cube_diag,
                    launch_timeout=3,
                    tcp_offset_pose=[0, 0, rc['tcp_offset'], 0, 0, 0],
                    payload_mass=None,
                    payload_cog=None,
                    joints_init=j_init,
                    joints_init_speed=1.05,
                    soft_real_time=False,
                    verbose=False,
                    receive_keys=None,
                    receive_latency=rc['robot_obs_latency']
                )
            else:
                raise NotImplementedError()
            robots.append(this_robot)

        for gc in grippers_config:
            # this_gripper = DHController(
            #             shm_manager=shm_manager,
            #             port=gc['gripper_port'],
            #             receive_latency=gc['gripper_obs_latency'],
            #             use_meters=True
            #         )
            this_gripper = SimGripperController(
                shm_manager=shm_manager,
                # hostname=gc['gripper_ip'],
                frequency=50,
                port=gc['gripper_port'],
                receive_latency=gc['gripper_obs_latency'],
                use_meters=True
            )
            grippers.append(this_gripper)

        self.camera = camera
        
        self.robots = robots
        self.robots_config = robots_config
        self.grippers = grippers
        self.grippers_config = grippers_config

        self.multi_cam_vis = multi_cam_vis
        self.frequency = frequency
        self.max_obs_buffer_size = max_obs_buffer_size
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        # timing
        self.camera_obs_latency = camera_obs_latency
        self.camera_down_sample_steps = camera_down_sample_steps
        self.robot_down_sample_steps = robot_down_sample_steps
        self.gripper_down_sample_steps = gripper_down_sample_steps
        self.camera_obs_horizon = camera_obs_horizon
        self.robot_obs_horizon = robot_obs_horizon
        self.gripper_obs_horizon = gripper_obs_horizon
        # recording
        self.output_dir = output_dir
        self.video_dir = video_dir
        self.replay_buffer = replay_buffer
        # temp memory buffers
        self.last_camera_data = None
        # recording buffers
        self.obs_accumulator = None
        self.action_accumulator = None

        self.start_time = None
        self.last_time_step = 0
    
    # ======== start-stop API =============
    @property
    def is_ready(self):
        ready_flag = self.camera.is_ready
        for robot in self.robots:
            ready_flag = ready_flag and robot.is_ready
        for gripper in self.grippers:
            ready_flag = ready_flag and gripper.is_ready
        return ready_flag
    
    def start(self, wait=True):
        self.camera.start(wait=False)
        for robot in self.robots:
            robot.start(wait=False)
        for gripper in self.grippers:
            gripper.start(wait=False)

        if self.multi_cam_vis is not None:
            self.multi_cam_vis.start(wait=False)
        if wait:
            self.start_wait()

    def stop(self, wait=True):
        self.end_episode()
        if self.multi_cam_vis is not None:
            self.multi_cam_vis.stop(wait=False)
        for robot in self.robots:
            robot.stop(wait=False)
        for gripper in self.grippers:
            gripper.stop(wait=False)
        self.camera.stop(wait=False)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.camera.start_wait()
        for robot in self.robots:
            robot.start_wait()
        for gripper in self.grippers:
            gripper.start_wait()
        if self.multi_cam_vis is not None:
            self.multi_cam_vis.start_wait()
    
    def stop_wait(self):
        for robot in self.robots:
            robot.stop_wait()
        for gripper in self.grippers:
            gripper.stop_wait()
        self.camera.stop_wait()
        if self.multi_cam_vis is not None:
            self.multi_cam_vis.stop_wait()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= async env API ===========
    def get_obs(self) -> dict:
        # np.set_printoptions(precision=10)
        """
        Timestamp alignment policy
        We assume the cameras used for obs are always [0, k - 1], where k is the number of robots
        All other cameras, find corresponding frame with the nearest timestamp
        All low-dim observations, interpolate with respect to 'current' time
        """

        "observation dict"
        assert self.is_ready

        # get data
        # 60 Hz, camera_calibrated_timestamp
        k = math.ceil(
            self.camera_obs_horizon * self.camera_down_sample_steps \
            * (30 / self.frequency)) + 2  # here 2 is adjustable, typically 1 should be enough
        #print('==>k  ', k, self.camera_obs_horizon, self.camera_down_sample_steps, self.frequency)
        
        t1 = time.monotonic()
        self.last_camera_data = self.camera.get(
            k=k, 
            out=self.last_camera_data)

        # both have more than n_obs_steps data
        last_robots_data = list()
        last_grippers_data = list()
        # 125/500 hz, robot_receive_timestamp
        t2 = time.monotonic()
        for robot in self.robots:
            last_robots_data.append(robot.get_all_state())
            #print(f"all state length: {len(robot.get_all_state()['ActualTCPPose'])}")
        t3 = time.monotonic()
        # 30 hz, gripper_receive_timestamp
        for gripper in self.grippers:
            last_grippers_data.append(gripper.get_all_state())
        t4 = time.monotonic()
        # print(f'get camera: {t2-t1}, get robot: {t3-t2}, get width: {t4-t3}')

        # select align_camera_idx
        num_obs_cameras = len(self.robots)
        align_camera_idx = None
        running_best_error = np.inf
   
        for camera_idx in range(num_obs_cameras):
            this_error = 0
            this_timestamp = self.last_camera_data[camera_idx]['timestamp'][-1]
            for other_camera_idx in range(num_obs_cameras):
                if other_camera_idx == camera_idx:
                    continue
                other_timestep_idx = -1
                while True:
                    if self.last_camera_data[other_camera_idx]['timestamp'][other_timestep_idx] < this_timestamp:
                        this_error += this_timestamp - self.last_camera_data[other_camera_idx]['timestamp'][other_timestep_idx]
                        break
                    other_timestep_idx -= 1
            if align_camera_idx is None or this_error < running_best_error:
                running_best_error = this_error
                align_camera_idx = camera_idx

        last_timestamp = self.last_camera_data[align_camera_idx]['timestamp'][-1]
        dt = 1 / self.frequency

        # align camera obs timestamps
        camera_obs_timestamps = last_timestamp - (
            np.arange(self.camera_obs_horizon)[::-1] * self.camera_down_sample_steps * dt)
        camera_obs = dict()
        for camera_idx, value in self.last_camera_data.items():
            this_timestamps = value['timestamp']
            this_idxs = list()
            for t in camera_obs_timestamps:
                nn_idx = np.argmin(np.abs(this_timestamps - t))
                # if np.abs(this_timestamps - t)[nn_idx] > 1.0 / 120 and camera_idx != 3:
                #     print('ERROR!!!  ', camera_idx, len(this_timestamps), nn_idx, (this_timestamps - t)[nn_idx-1: nn_idx+2])
                this_idxs.append(nn_idx)
            # remap key
            camera_obs[f'camera_{camera_idx}'] = value['color'][this_idxs]

        # obs_data to return (it only includes camera data at this stage)
        obs_data = dict(camera_obs)

        # include camera timesteps
        obs_data['timestamp'] = camera_obs_timestamps

        # align robot obs
        robot_obs_timestamps = last_timestamp - (
            np.arange(self.robot_obs_horizon)[::-1] * self.robot_down_sample_steps * dt)
        for robot_idx, last_robot_data in enumerate(last_robots_data):
            robot_pose_interpolator = PoseInterpolator(
                t=last_robot_data['robot_timestamp'], 
                x=last_robot_data['ActualTCPPose'])
            robot_pose = robot_pose_interpolator(robot_obs_timestamps)
            robot_obs = {
                f'robot{robot_idx}_eef_pos': robot_pose[...,:3],
                f'robot{robot_idx}_eef_rot_axis_angle': robot_pose[...,3:]
            }
            # update obs_data
            obs_data.update(robot_obs)

        # align gripper obs
        gripper_obs_timestamps = last_timestamp - (
            np.arange(self.gripper_obs_horizon)[::-1] * self.gripper_down_sample_steps * dt)
        for robot_idx, last_gripper_data in enumerate(last_grippers_data):
            # align gripper obs
            gripper_interpolator = get_interp1d(
                t=last_gripper_data['gripper_timestamp'],
                x=last_gripper_data['gripper_position'][...,None]
            )
            gripper_obs = {
                f'robot{robot_idx}_gripper_width': gripper_interpolator(gripper_obs_timestamps)
            }

            # update obs_data
            obs_data.update(gripper_obs)
        # print(f"obs_data: {obs_data['timestamp']}")

        # # accumulate obs
        if self.obs_accumulator is not None:
            self.obs_accumulator.put(
                data={
                    'camera': np.empty(obs_data['timestamp'].shape),
                },
                timestamps=obs_data['timestamp']
            )
            for robot_idx, last_robot_data in enumerate(last_robots_data):
                self.obs_accumulator.put(
                    data={
                        f'robot{robot_idx}_eef_pose': last_robot_data['ActualTCPPose'],
                        # f'robot{robot_idx}_joint_pos': last_robot_data['ActualQ'],
                        # f'robot{robot_idx}_joint_vel': last_robot_data['ActualQd'],
                    },
                    timestamps=last_robot_data['robot_timestamp']
                )
                # print(f"robot_data: {last_robot_data['robot_timestamp']}")
            for robot_idx, last_gripper_data in enumerate(last_grippers_data):
                self.obs_accumulator.put(
                    data={
                        f'robot{robot_idx}_gripper_width': last_gripper_data['gripper_position'][...,None]
                    },
                    timestamps=last_gripper_data['gripper_timestamp']
                )
                #print(f"last_gripper_data: {last_gripper_data['gripper_timestamp'][:10]}")

        return obs_data
    
    def exec_actions(self, 
            actions: np.ndarray, 
            timestamps: np.ndarray,
            compensate_latency=False):
        assert self.is_ready
        if not isinstance(actions, np.ndarray):
            actions = np.array(actions)
        if not isinstance(timestamps, np.ndarray):
            timestamps = np.array(timestamps)

        # convert action to pose
        receive_time = time.time()
        is_new = timestamps > receive_time
        new_actions = actions[is_new]
        new_timestamps = timestamps[is_new]
        print(f'exec action timestamp:{new_timestamps}, new_actions: {new_actions}')

        assert new_actions.shape[1] // len(self.robots) == 7
        assert new_actions.shape[1] % len(self.robots) == 0

        # schedule waypoints
        for i in range(len(new_actions)):
            for robot_idx, (robot, gripper, rc, gc) in enumerate(zip(self.robots, self.grippers, self.robots_config, self.grippers_config)):
                r_latency = rc['robot_action_latency'] if compensate_latency else 0.0
                g_latency = gc['gripper_action_latency'] if compensate_latency else 0.0
                r_actions = new_actions[i, 7 * robot_idx + 0: 7 * robot_idx + 6]
                g_actions = new_actions[i, 7 * robot_idx + 6]
                # print(f"[bimanual_umi_sim_env] robot action: {r_actions}")
                robot.schedule_waypoint(
                    pose=r_actions,
                    target_time=new_timestamps[i] - r_latency
                )
                gripper.schedule_waypoint(
                    pos=g_actions,
                    target_time=new_timestamps[i] - g_latency
                )

        # print(new_timestamps)
        # record actions
        if self.action_accumulator is not None:
            self.action_accumulator.put(
                new_actions,
                new_timestamps
            )
    
    def get_robot_state(self):
        return [robot.get_state() for robot in self.robots]
    
    def get_gripper_state(self):
        return [gripper.get_state() for gripper in self.grippers]

    # recording API
    def start_episode(self, start_time=None):
        "Start recording and return first obs"
        if start_time is None:
            start_time = time.time()
        # start_time += 1
        self.start_time = start_time

        assert self.is_ready

        # prepare recording stuff
        episode_id = self.replay_buffer.n_episodes
        this_video_dir = self.video_dir.joinpath(str(episode_id))
        this_video_dir.mkdir(parents=True, exist_ok=True)
        n_cameras = self.camera.n_cameras
        video_paths = list()
        for i in range(n_cameras):
            video_paths.append(
                str(this_video_dir.joinpath(f'{i}.mp4').absolute()))
        
        print(f'start_time: {start_time}')
        # start recording on camera
        self.camera.restart_put(start_time=start_time)
        self.camera.start_recording(video_path=video_paths, start_time=start_time)

        # create accumulators
        self.obs_accumulator = ObsAccumulator()
        self.action_accumulator = TimestampActionAccumulator(
            start_time=start_time,
            dt=1/self.frequency
        )
        print(f'Episode {episode_id} started!')
    
    def end_episode(self):
        "Stop recording"
        assert self.is_ready
        
        # stop video recorder
        self.camera.stop_recording()

        # TODO
        if self.obs_accumulator is not None:
            # recording
            assert self.action_accumulator is not None

            # Since the only way to accumulate obs and action is by calling
            # get_obs and exec_actions, which will be in the same thread.
            # We don't need to worry new data come in here.
            end_time = float('inf')
            dt = 1 /30
            minkey = None
            for key, value in self.obs_accumulator.timestamps.items():
                print(f'key : {key}, duration: {value[-1]-value[0]},start:{value[0]:.10f}, end:{value[-1]:.10f}')
                if end_time > value[-1]:
                   minkey = key
                   end_time = value[-1]
                #end_time = min(end_time, value[-1])
            video_n_frame = math.floor((end_time - self.start_time)/dt)-1
            timestamps = np.arange(video_n_frame)*float(dt)+self.start_time
            print(minkey, end_time)
            # print(timestamps)
                
            # end_time = min(end_time, self.action_accumulator.timestamps[-1])

            # actions = self.action_accumulator.actions
            # action_timestamps = self.action_accumulator.timestamps
            n_steps = 1

            #if np.sum(self.obs_accumulator.timestamps['camera'] <= end_time) > 0:
            #    n_steps = np.nonzero(self.obs_accumulator.timestamps['camera'] <= end_time)[0][-1]+1

            if n_steps > 0:
                #timestamps = self.obs_accumulator.timestamps['camera'][:n_steps]
                episode = {
                    'timestamp': timestamps,
                    #'action': actions[:n_steps],
                }
                for robot_idx in range(len(self.robots)):
                    # print (f"gripper: {self.obs_accumulator.data['robot0_gripper_width']}, pose: {self.obs_accumulator.timestamps['robot0_eef_pose']}")
                    robot_pose_interpolator = PoseInterpolator(
                        t=np.array(self.obs_accumulator.timestamps[f'robot{robot_idx}_eef_pose']),
                        x=np.array(self.obs_accumulator.data[f'robot{robot_idx}_eef_pose'])
                    )
                    robot_pose = robot_pose_interpolator(timestamps)
                    episode[f'robot{robot_idx}_eef_pos'] = robot_pose[:,:3]
                    episode[f'robot{robot_idx}_eef_rot_axis_angle'] = robot_pose[:,3:]
                    # joint_pos_interpolator = get_interp1d(
                    #     np.array(self.obs_accumulator.timestamps[f'robot{robot_idx}_joint_pos']),
                    #     np.array(self.obs_accumulator.data[f'robot{robot_idx}_joint_pos')
                    # )
                    # joint_vel_interpolator = get_interp1d(
                    #     np.array(self.obs_accumulator.timestamps[f'robot{robot_idx}_joint_vel']),
                    #     np.array(self.obs_accumulator.data[f'robot{robot_idx}_joint_vel'])
                    # )
                    # episode[f'robot{robot_idx}_joint_pos'] = joint_pos_interpolator(timestamps)
                    # episode[f'robot{robot_idx}_joint_vel'] = joint_vel_interpolator(timestamps)

                    gripper_interpolator = get_interp1d(
                        t=np.array(self.obs_accumulator.timestamps[f'robot{robot_idx}_gripper_width']),
                        x=np.array(self.obs_accumulator.data[f'robot{robot_idx}_gripper_width'])
                    )
                    episode[f'robot{robot_idx}_gripper_width'] = gripper_interpolator(timestamps)

                self.replay_buffer.add_episode(episode, compressors='disk')
                episode_id = self.replay_buffer.n_episodes - 1
                print(f'Episode {episode_id} saved!')
            
            self.obs_accumulator = None
            self.action_accumulator = None

    def drop_episode(self):
        self.end_episode()
        self.replay_buffer.drop_episode()
        episode_id = self.replay_buffer.n_episodes
        this_video_dir = self.video_dir.joinpath(str(episode_id))
        if this_video_dir.exists():
            shutil.rmtree(str(this_video_dir))
        print(f'Episode {episode_id} dropped!')
