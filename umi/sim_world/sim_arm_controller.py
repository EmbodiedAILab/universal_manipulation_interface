import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import scipy.interpolate as si
import scipy.spatial.transform as st
import numpy as np
from umi.sim_world.ros_control_interface import ControlInterface
from umi.sim_world.ros_receive_interface import ReceiveInterface
from umi.sim_world.zmq_receiver import ZmqSubcriber
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.common.precise_sleep import precise_wait

# global RESET_VALUE

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2


class SimArmController(mp.Process):
    """
    To ensure sending command to the robot with predictable latency
    this controller need its separate process (due to python GIL)
    """


    def __init__(self,
            shm_manager: SharedMemoryManager, 
            robot_ip, 
            frequency=125, 
            lookahead_time=0.1, 
            gain=300,
            max_pos_speed=0.25, # 5% of max speed
            max_rot_speed=0.16, # 5% of max speed
            launch_timeout=3,
            tcp_offset_pose=None,
            payload_mass=None,
            payload_cog=None,
            joints_init=None,
            joints_init_speed=1.05,
            soft_real_time=False,
            verbose=True,
            receive_keys=None,
            get_max_k=None,
            receive_latency=0.0
            ):
        """
        frequency: CB2=125, UR3e=500
        lookahead_time: [0.03, 0.2]s smoothens the trajectory with this lookahead time
        gain: [100, 2000] proportional gain for following target position
        max_pos_speed: m/s
        max_rot_speed: rad/s
        tcp_offset_pose: 6d pose
        payload_mass: float
        payload_cog: 3d position, center of gravity
        soft_real_time: enables round-robin scheduling and real-time priority
            requires running scripts/rtprio_setup.sh before hand.

        """
        # verify
        assert 0 < frequency <= 500
        assert 0.03 <= lookahead_time <= 0.2
        assert 100 <= gain <= 2000
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        if tcp_offset_pose is not None:
            tcp_offset_pose = np.array(tcp_offset_pose)
            assert tcp_offset_pose.shape == (6,)
        if payload_mass is not None:
            assert 0 <= payload_mass <= 5
        if payload_cog is not None:
            payload_cog = np.array(payload_cog)
            assert payload_cog.shape == (3,)
            assert payload_mass is not None
        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)

        super().__init__(name="SimArmController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.lookahead_time = lookahead_time
        self.gain = gain
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.launch_timeout = launch_timeout
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose
        self.reset_value = False
        self.zmq_sub = ZmqSubcriber(shm_manager=shm_manager)

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # build input queue
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # build ring buffer
        # ros_r = ReceiveInterface()
        # if receive_keys is None:
        #     receive_keys = [
        #         'ActualTCPPose',
        #         # 'ActualTCPSpeed',
        #         'ActualQ',
        #         # 'ActualQd',

        #         # 'TargetTCPPose',
        #         # 'TargetTCPSpeed',
        #         # 'TargetQ',
        #         # 'TargetQd'
        #     ]
        # self.wait_for_valid_data(ros_r)

        # self.wait_for_valid_data()
        # example = dict()
        # # for key in receive_keys:
        # #     example[key] = np.array(getattr(ros_r, 'get'+key)())
        # example['ActualTCPPose'] = self.ros_r.get_state()['ActualTCPPose']
        # example['ActualQ'] = self.ros_r.get_state()['ActualQ']
        # example['robot_receive_timestamp'] = time.time()
        # example['robot_timestamp'] = time.time()
        # print(f'get example: {example}')
        # ring_buffer = SharedMemoryRingBuffer.create_from_examples(
        #     shm_manager=shm_manager,
        #     examples=example,
        #     get_max_k=get_max_k,
        #     get_time_budget=0.2,
        #     put_desired_frequency=frequency
        # )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = None
        self.receive_keys = receive_keys
        self.shm_manager = shm_manager
        self.get_max_k = get_max_k
        self.frequency = frequency

    
    # ========= launch method ===========
    def start(self, wait=True):
        self.zmq_sub.start()
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[SimArmController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        self.zmq_sub.stop()
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
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
        
    def servoJ(self):  
        print('call servoJ')
        self.reset_value=True
     
    # ========= command methods ============
    def servoL(self, pose, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert(duration >= (1/self.frequency))
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)
    
    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'target_time': target_time
        }
        self.input_queue.put(message)

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.zmq_sub.get_eef_state(out=out)
        else:
            return self.zmq_sub.get_eef_state(k=k,out=out)
    
    def get_all_state(self):
        return self.zmq_sub.get_all_state_eef()

    # ========= main loop in process ============
    def run(self):
        # self.wait_for_valid_data()

        # example = dict()
        # # for key in receive_keys:
        # #     example[key] = np.array(getattr(ros_r, 'get'+key)())
        # example['ActualTCPPose'] = self.ros_r.get_eef_state()['ActualTCPPose']
        # # example['ActualQ'] = self.ros_r.get_state()['ActualQ']
        # example['robot_receive_timestamp'] = time.time()
        # example['robot_timestamp'] = time.time()
        # print(f'get example: {example}')
        # self.ring_buffer = SharedMemoryRingBuffer.create_from_examples(
        #     shm_manager=self.shm_manager,
        #     examples=example,
        #     get_max_k=self.get_max_k,
        #     get_time_budget=0.2,
        #     put_desired_frequency=self.frequency
        # )
    
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))

        # start rtde
        robot_ip = self.robot_ip
        print('robot_ip', robot_ip)
        ros_c = ControlInterface()
        # ros_r = ReceiveInterface()


        try:
            if self.verbose:
                print(f"[SimArmController] Connect to robot: {robot_ip}")

            # init pose
            # if self.joints_init is not None:
            #     # assert rtde_c.moveJ(self.joints_init, self.joints_init_speed, 1.4)
            #     assert ros_c.moveJ(self.joints_init)

            # main loop
            dt = 1. / self.frequency
            # self.wait_for_valid_data(ros_r)
            # curr_pose = self.ros_r.getActualTCPPose()
            curr_pose = self.zmq_sub.get_eef_state()['ActualTCPPose']
            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            # last_loop_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )
            
            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                # send command to robot
                t_now = time.monotonic()
                # print(f"hz: {1 / (t_now - last_loop_time)}")
                # last_loop_time = t_now
                pose_command = pose_interp(t_now)
                # vel = 0.5
                # acc = 0.5
                #print(f"[sim_arm_controller] pose: {pose_command}")
                assert ros_c.servoL(pose_command)
                
                # update robot state
                # state = dict()
                # # for key in self.receive_keys:
                # #     state[key] = np.array(getattr(ros_r, 'get'+key)())
                # state['ActualTCPPose'] = self.ros_r.get_eef_state()['ActualTCPPose']
                # # state['ActualQ'] = self.ros_r.get_state()['ActualQ']
                # t_recv = time.time()
                # state['robot_receive_timestamp'] = t_recv
                # state['robot_timestamp'] = t_recv - self.receive_latency
                # self.ring_buffer.put(state)

                # fetch command from queue
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

                    if cmd == Command.STOP.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinouity 
                        # and cause jittery robot behavior.
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[SimArmController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        # t_now2 = time.monotonic()
                        # pose_command = pose_interp(t_now2)
                        # print(f'target_time: {target_time}, times: {t}, curr_time: {curr_time}, target_pose: {target_pose}, pose_command: {pose_command}')
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[SimArmController] Actual frequency {1/(time.monotonic() - t_now)}")
        except Exception as e:
            print(f"[SimArmController] Exception: {e}")
        finally:
            self.ready_event.set()
            ros_c.cleanup()
            # ros_r.cleanup()

            if True:
                print(f"[SimArmController] Disconnected from robot: {robot_ip}")
    
    def wait_for_valid_data(self, timeout=10, check_interval=0.1):
        start_time = time.time()
        while True:
            if 'ActualTCPPose' in self.ros_r.get_eef_state().keys():
                return

            if time.time() - start_time > timeout:
                raise TimeoutError("wait timeout, check zmq data")
            
            time.sleep(check_interval)
