# %%
import sys
import os

# ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
# sys.path.append(ROOT_DIR)
# #print('root', ROOT_DIR)
# os.chdir(ROOT_DIR)
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')


# %%
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import scipy.spatial.transform as st
from umi.sim_world.bimanual_umi_sim_env import BimanualUmiSimEnv
from umi.real_world.spacemouse_shared_memory import Spacemouse
from umi.common.precise_sleep import precise_wait
from umi.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)
from umi.common.pose_util import pose_to_mat, mat_to_pose
from umi.real_world.spacemouse_shared_memory import Spacemouse


def solve_table_collision(ee_pose, gripper_width, height_threshold):
    finger_thickness = 25.5 / 1000
    keypoints = list()
    for dx in [-1, 1]:
        for dy in [-1, 1]:
            keypoints.append((dx * gripper_width / 2, dy * finger_thickness / 2, 0))
    keypoints = np.asarray(keypoints)
    rot_mat = st.Rotation.from_rotvec(ee_pose[3:6]).as_matrix()
    transformed_keypoints = np.transpose(rot_mat @ np.transpose(keypoints)) + ee_pose[:3]
    delta = max(height_threshold - np.min(transformed_keypoints[:, 2]), 0)
    ee_pose[2] += delta

def solve_sphere_collision(ee_poses, robots_config):
    num_robot = len(robots_config)
    this_that_mat = np.identity(4)
    this_that_mat[:3, 3] = np.array([0, 0.89, 0]) # TODO: very hacky now!!!!

    for this_robot_idx in range(num_robot):
        for that_robot_idx in range(this_robot_idx + 1, num_robot):
            this_ee_mat = pose_to_mat(ee_poses[this_robot_idx][:6])
            this_sphere_mat_local = np.identity(4)
            this_sphere_mat_local[:3, 3] = np.asarray(robots_config[this_robot_idx]['sphere_center'])
            this_sphere_mat_global = this_ee_mat @ this_sphere_mat_local
            this_sphere_center = this_sphere_mat_global[:3, 3]

            that_ee_mat = pose_to_mat(ee_poses[that_robot_idx][:6])
            that_sphere_mat_local = np.identity(4)
            that_sphere_mat_local[:3, 3] = np.asarray(robots_config[that_robot_idx]['sphere_center'])
            that_sphere_mat_global = this_that_mat @ that_ee_mat @ that_sphere_mat_local
            that_sphere_center = that_sphere_mat_global[:3, 3]

            distance = np.linalg.norm(that_sphere_center - this_sphere_center)
            threshold = robots_config[this_robot_idx]['sphere_radius'] + robots_config[that_robot_idx]['sphere_radius']
            # print(that_sphere_center, this_sphere_center)
            if distance < threshold:
                half_delta = (threshold - distance) / 2
                normal = (that_sphere_center - this_sphere_center) / distance
                this_sphere_mat_global[:3, 3] -= half_delta * normal
                that_sphere_mat_global[:3, 3] += half_delta * normal
                
                ee_poses[this_robot_idx][:6] = mat_to_pose(this_sphere_mat_global @ np.linalg.inv(this_sphere_mat_local))
                ee_poses[that_robot_idx][:6] = mat_to_pose(np.linalg.inv(this_that_mat) @ that_sphere_mat_global @ np.linalg.inv(that_sphere_mat_local))

@click.command()
@click.option('--output', '-o', required=True)
@click.option('--robot_ip', default='172.24.95.9')
@click.option('--gripper_ip', default='172.24.95.17')
# @click.option('--rqobot_ip', default='172.24.95.8')
# @click.option('--gripper_ip', default='172.24.95.18')
@click.option('--vis_camera_idx', default=0, type=int)
@click.option('--init_joints', '-j', is_flag=True, default=False)
@click.option('-gs', '--gripper_speed', type=float, default=0.2)
def main(output, robot_ip, gripper_ip, vis_camera_idx, init_joints, gripper_speed):
            # robots_config, # list of dict[{robot_type: 'ur5', robot_ip: XXX, obs_latency: 0.0001, action_latency: 0.1, tcp_offset: 0.21}]
            # grippers_config, # list of dict[{gripper_ip: XXX, gripper_port: 1000, obs_latency: 0.01, action_latency: 0.1}]

    robots_config = [
        {
            'robot_type': 'ur5e',
            'robot_ip': '192.168.2.152',
            'robot_obs_latency': 0.0001, 'robot_action_latency': 0.1, 'tcp_offset': 0.235,
            'height_threshold': 0.017,
            'sphere_radius': 0.13, 'sphere_center': [0, 0, -0.185],
            # 'height_threshold': -0.075
        },
        # {
        #     'robot_type': 'ur5',
        #     'robot_ip': '172.24.95.9',
        #     'robot_obs_latency': 0.0001, 'robot_action_latency': 0.1, 'tcp_offset': 0.235,
        #     'height_threshold': 0.022,
        #     'sphere_radius': 0.13, 'sphere_center': [0, 0, -0.185],
        # }
    ]
    grippers_config = [
        {
            "robot_type": "dh",
            "gripper_port": "/dev/ttyUSBDH_", "gripper_obs_latency": 0.01, "gripper_action_latency": 0.1
        }
    ]

    max_gripper_width = 100

    frequency = 30
    command_latency = 1/100
    dt = 1/frequency
    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager) as sm,\
            KeystrokeCounter() as key_counter, \
            BimanualUmiSimEnv(
                output_dir=output,
                robots_config=robots_config,
                grippers_config=grippers_config,
                camera_reorder=[1, 2, 3, 0],
                obs_image_resolution=(1280,720),
                frequency=frequency,
                init_joints=init_joints,
                enable_multi_cam_vis=True,
                # record_raw_video=True,
                # thread_per_video=3,
                # video_crf=21,
                shm_manager=shm_manager
            ) as env:
            cv2.setNumThreads(1)

            time.sleep(3.0)
            print('Ready!')
            states = env.get_robot_state()
            target_pose = np.stack([s['ActualTCPPose'] for s in states])
            gripper_target_pos = [max_gripper_width] * target_pose.shape[0]
            control_robot_idx_list = [0]
            obs = env.get_obs()
            # visualize
            vis_img = obs[f'camera_{vis_camera_idx}'][-1,:,:,::-1].copy()
            episode_id = env.replay_buffer.n_episodes
            text = f'Episode: {episode_id}'
            cv2.putText(
                vis_img,
                text,
                (10,30),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1,
                thickness=2,
                color=(255,255,255)
            )
            cv2.imshow('default', vis_img)
            cv2.pollKey()

            t_start = time.monotonic()
            # print(f't_start: {t_start}')
            iter_idx = 0
            stop = False
            is_recording = False
            while not stop:
                # calculate timing
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + 2*dt

                t1 = time.monotonic()
                # pump obs
                obs = env.get_obs()
                t2 = time.monotonic()
                # print(f'delta: {t_command_target-t2}, duration obs: {t2-t1}')

                # handle key presses
                press_events = key_counter.get_press_events()
                for key_stroke in press_events:
                    # if key_stroke != None:
                    #     print(key_stroke)
                    if key_stroke == KeyCode(char='q'):
                        stop = True
                    elif key_stroke == KeyCode(char='c'):
                        env.start_episode(t_start + (iter_idx + 2) * dt - time.monotonic() + time.time())
                        key_counter.clear()
                        is_recording = True
                        print('Recording!')
                    elif key_stroke == KeyCode(char='s'):
                        env.end_episode()
                        key_counter.clear()
                        is_recording = False
                        print('Stopped.')
                    elif key_stroke == KeyCode(char='e'):
                            # Next episode
                        if match_episode is not None:
                            match_episode = min(match_episode + 1, env.replay_buffer.n_episodes-1)
                    elif key_stroke == KeyCode(char='w'):
                            # Prev episode
                        if match_episode is not None:
                            match_episode = max(match_episode - 1, 0)
                    elif key_stroke == Key.backspace:
                        if click.confirm('Are you sure to drop an episode?'):
                            env.drop_episode()
                            key_counter.clear()
                            is_recording = False
                        # delete
                    elif key_stroke == KeyCode(char='a'):
                        control_robot_idx_list = list(range(target_pose.shape[0]))
                    elif key_stroke == KeyCode(char='1'):
                        control_robot_idx_list = [0]
                    elif key_stroke == KeyCode(char='2'):
                        control_robot_idx_list = [1]

                stage = key_counter[Key.space]
                # print(f'duration key: {time.monotonic()-t2}')

                # visualize
                vis_img = obs[f'camera_{vis_camera_idx}'][-1,:,:,::-1].copy()
                episode_id = env.replay_buffer.n_episodes
                text = f'Episode: {episode_id}, Stage: {stage}'
                if is_recording:
                    text += ', Recording!'
                cv2.putText(
                    vis_img,
                    text,
                    (10,30),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1,
                    thickness=2,
                    color=(255,255,255)
                )

                cv2.imshow('default', vis_img)
                cv2.pollKey()
                t3 = time.monotonic()
                # print(f'delta: {t_command_target-t3}, duration cv: {t3-t2}')

                precise_wait(t_sample)
                # get teleop command
                sm_state = sm.get_motion_state_transformed()
                print(sm_state)
                dpos = sm_state[:3] * (env.max_pos_speed / frequency)
                drot_xyz = sm_state[3:] * (env.max_rot_speed / frequency)

                drot = st.Rotation.from_euler('xyz', drot_xyz)
                for robot_idx in control_robot_idx_list:
                    target_pose[robot_idx, :3] += dpos
                    target_pose[robot_idx, 3:] = (drot * st.Rotation.from_rotvec(
                        target_pose[robot_idx, 3:])).as_rotvec()
                
                dpos = 0
                if sm.is_button_pressed(0):
                    # close gripper
                    dpos = -gripper_speed / frequency
                if sm.is_button_pressed(1):
                    dpos = gripper_speed / frequency
                for robot_idx in control_robot_idx_list:
                    gripper_target_pos[robot_idx] = np.clip(gripper_target_pos[robot_idx] + dpos, 0, max_gripper_width)


                # solve collision with table
                for robot_idx in control_robot_idx_list:
                    solve_table_collision(
                        ee_pose=target_pose[robot_idx],
                        gripper_width=gripper_target_pos[robot_idx],
                        height_threshold=robots_config[robot_idx]['height_threshold'])

                #solve collison between two robots
                solve_sphere_collision(
                    ee_poses=target_pose,
                    robots_config=robots_config
                )

                action = np.zeros((7 * target_pose.shape[0],))

                for robot_idx in range(target_pose.shape[0]):
                    action[7 * robot_idx + 0: 7 * robot_idx + 6] = target_pose[robot_idx]
                    action[7 * robot_idx + 6] = gripper_target_pos[robot_idx]

                # execute teleop command
                t4 = time.monotonic()
                # print(f't_command_target: {t_command_target}, delta: {t_command_target-t4}, duration: {t4-t3}')
                env.exec_actions(
                    actions=[action], 
                    timestamps=[t_command_target-time.monotonic()+time.time()],
                    compensate_latency=False)
                precise_wait(t_cycle_end)
                iter_idx += 1

# %%
if __name__ == '__main__':
    main()
