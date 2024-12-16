import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import SharedMemoryQueue, Empty
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.precise_sleep import precise_wait
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from umi.sim_world.ros_control_interface import GripperControlInterface
from umi.sim_world.zmq_receiver import ZmqSubcriber


class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2


class SimGripperController(mp.Process):
    def __init__(
        self,
        shm_manager: SharedMemoryManager,
        gripper_ip='localhost',
        band_rate=115200,
        port="/dev/ttyUSBDH_",
        frequency=10,
        max_speed=0.5,
        max_width=0.08,
        min_width=0.0,
        grasp_threshold=0.0,
        grasp_offset=0.0,
        max_force=140,
        get_max_k=None,
        command_queue_size=1024,
        launch_timeout=3,
        receive_latency=0.0,
        use_meters=True,
        verbose=False,
    ):
        super().__init__(name="SimGripperController")
        self.gripper_ip=gripper_ip
        self.band_rate = band_rate
        self.port = port
        self.frequency = frequency
        self.max_speed = max_speed
        self.max_width = max_width
        self.min_width = min_width
        self.grasp_threshold = grasp_threshold
        self.grasp_offset = grasp_offset
        self.max_force = max_force
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.scale = 1.0 if use_meters else 1000.0
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)

        # build input queue
        example = {
            "cmd": Command.SCHEDULE_WAYPOINT.value,
            "target_pos": self.max_width,
            "target_time": 0.0,
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager, examples=example, buffer_size=command_queue_size
        )

        self.zmq_sub = ZmqSubcriber(shm_manager=shm_manager, zmq_host=self.gripper_ip)

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        # self.ring_buffer = ring_buffer

    # ========= launch method ===========
    def start(self, wait=True):
        self.zmq_sub.start()
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[SimGripperController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {"cmd": Command.SHUTDOWN.value}
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

    # ========= command methods ============
    def schedule_waypoint(self, pos: float, target_time: float):
        message = {
            "cmd": Command.SCHEDULE_WAYPOINT.value,
            "target_pos": pos,
            "target_time": target_time,
        }
        self.input_queue.put(message)

    def restart_put(self, start_time):
        self.input_queue.put(
            {"cmd": Command.RESTART_PUT.value, "target_time": start_time}
        )

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.zmq_sub.get_gripper_state(out=out)
        else:
            return self.zmq_sub.get_gripper_state(k=k,out=out)

    def get_all_state(self):
        return self.zmq_sub.get_all_state_gripper()
    
    # def run(self):
    #     while True:
    #         time.sleep(10.0)

    # ========= main loop in process ============
    def run(self):
        # start connection
        try:             
            ros_c = GripperControlInterface(zmq_host=self.gripper_ip)
            # get initial
            curr_pos = self.zmq_sub.get_gripper_state()['gripper_position']
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t], poses=[[curr_pos, 0, 0, 0, 0, 0]]
            )

            keep_running = True
            t_start = time.monotonic()
            iter_idx = 0
            target_pos = curr_pos
            target_vel = self.max_speed
            picked = False
            while keep_running:
                # command gripper
                t_now = time.monotonic()
                dt = 1 / self.frequency
                t_target = t_now
                target_pos = pose_interp(t_target)[0]
                target_vel = (target_pos - pose_interp(t_target - dt)[0]) / dt
                origin_target_pos = target_pos
                # print("target_pos", target_pos)
                ros_c.setGripperTargetPos(target_pos)    

                # fetch command from queue
                try:
                    commands = self.input_queue.get_all()
                    n_cmd = len(commands["cmd"])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command["cmd"]

                    if cmd == Command.SHUTDOWN.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pos = command["target_pos"] * self.scale
                        if target_pos < self.grasp_threshold:
                            target_pos = target_pos - self.grasp_offset
                        target_time = command["target_time"]
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=[target_pos, 0, 0, 0, 0, 0],
                            time=target_time,
                            # max_pos_speed=self.max_speed,
                            # max_rot_speed=self.max_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = target_time
                    elif cmd == Command.RESTART_PUT.value:
                        t_start = (
                            command["target_time"] - time.time() + time.monotonic()
                        )
                        iter_idx = 1
                    else:
                        keep_running = False
                        break

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                # regulate frequency
                dt = 1 / self.frequency
                t_end = t_start + dt * iter_idx
                precise_wait(t_end=t_end, time_func=time.monotonic)

        finally:
            ros_c.cleanup()
            self.ready_event.set()
            if True:
                print(f"[SimGripperController] Disconnected from robot")
