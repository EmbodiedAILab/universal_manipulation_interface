import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
from umi.shared_memory.shared_memory_queue import SharedMemoryQueue, Empty
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.precise_sleep import precise_wait
from umi.common.status_interpolator import StatusInterpolator
from umi.sim_world.ros_control_interface import VacuumControlInterface
from umi.sim_world.zmq_receiver import ZmqSubcriber


class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2


class SimVacuumController(mp.Process):
    def __init__(
        self,
        shm_manager: SharedMemoryManager,
        frequency=10,  # 手册没查询到，一般20-40Hz，跟wsg夹爪保持一致
        # max_speed=0.5, #0.07273,  # 双侧夹爪相对的速度，单位m/s (根据手册，打开时间约1.1s)
        # max_width=0.08,  # 夹爪的最大打开宽度，单位m
        # max_force=140,  # 夹爪的最大力
        get_max_k=None,
        command_queue_size=1024,
        launch_timeout=3,
        receive_latency=0.0,
        verbose=False,
    ):
        super().__init__(name="SimGripperController")
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)

        # build input queue
        example = {
            "cmd": Command.SCHEDULE_WAYPOINT.value,
            "target_pos": 100.0,
            "target_time": 0.0,
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager, examples=example, buffer_size=command_queue_size
        )

        self.zmq_sub = ZmqSubcriber(shm_manager=shm_manager)
        self.ready_event = mp.Event()
        self.input_queue = input_queue

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
            return self.zmq_sub.get_vacuum_state(out=out)
        else:
            return self.zmq_sub.get_vacuum_state(k=k,out=out)

    def get_all_state(self):
        return self.zmq_sub.get_all_state_vacuum()

    # ========= main loop in process ============
    def run(self):
        # start connection
        try:             
            ros_c = VacuumControlInterface()
            # get initial
            status = self.zmq_sub.get_vacuum_state()['vacuum_status']
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            # TODO
            status_interp = StatusInterpolator(
                times=[curr_t], status=[status]
            )

            keep_running = True
            t_start = time.monotonic()
            iter_idx = 0
            while keep_running:
                # command gripper
                t_now = time.monotonic()
                t_target = t_now
                target_status = status_interp(t_target)
                # print("target_pos", target_pos)
                ros_c.setVacuumTargetPos(target_status)   #TODO 夹爪控制待调试

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
                        target_status = command["target_pos"]
                        target_time = command["target_time"]
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now
                        status_interp = status_interp.schedule_waypoint(
                            status=target_status,
                            time=target_time,
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
            if self.verbose:
                print(f"[SimVacuumController] Disconnected from robot")
