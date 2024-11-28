import sys
import os
from datetime import datetime

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import pathlib
import shutil

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from umi.common.interpolation_util import get_interp1d, PoseInterpolator
from diffusion_policy.common.replay_buffer import ReplayBuffer


def csv2eposide(file_path):
    csv_path = file_path
    # if not csv_path.is_file():
    #     print(f"Skipping {video_dir.name}, no camera_trajectory.csv.")
    #     dropped_camera_count[row['camera_serial']] += 1
    #     continue

    csv_df = pd.read_csv(csv_path)
    csv_df.set_index('frame_idx', inplace=True)
    # print(csv_df.iloc[0])
    pose_df = csv_df.loc[~csv_df.index.duplicated(), :]
    pos = pose_df[['x', 'y', 'z']].to_numpy()
    rot_quat_xyzw = pose_df[['q_x', 'q_y', 'q_z', 'q_w']].to_numpy()
    rot = Rotation.from_quat(rot_quat_xyzw)
    # print(pos.shape, rot.as_eule('xyz').shape)
    pose = np.hstack((pos, rot.as_rotvec()))
    print(pose)

    # select aligned frames
    # df = df_reindexed.iloc[start_frame_idx: start_frame_idx + n_frames]

    gripper_path = file_path.with_name(file_path.name.replace('pose_', 'gripper_'))
    csv_df = pd.read_csv(gripper_path)
    csv_df.set_index('frame_idx', inplace=True)
    # print(gripper_df.loc[gripper_df.index.duplicated(), :])
    gripper_df = csv_df.loc[~csv_df.index.duplicated(), :]
    # full_index = pd.RangeIndex(start=0, stop=csv_df.index.max() + 1, step=1)
    # gripper_df = csv_df.reindex(full_index)
    # gripper_df.iloc[0] = gripper_df.iloc[1]
    # gripper_df.iloc[0]['timestamp'] = 0

    timestamps = pose_df['timestamp'].to_numpy()
    if timestamps[-1] > gripper_df.iloc[0]['timestamp']:
        timestamps = gripper_df['timestamp'].to_numpy()
        n_steps = len(timestamps)
        episode = {
            'timestamp': np.array(timestamps),
            # 'action': actions[:n_steps],
        }

        robot_pose_interpolator = PoseInterpolator(
            t=pose_df['timestamp'].to_numpy(),
            x=pose
        )
        robot_pose = robot_pose_interpolator(timestamps)
        episode['robot0_eef_pos'] = robot_pose[:, :3]
        episode['robot0_eef_rot_axis_angle'] = robot_pose[:, 3:]

        width = gripper_df['width'].to_numpy()
        width = width[:, np.newaxis]
        gripper_interpolator = get_interp1d(
            t=gripper_df['timestamp'].to_numpy(),
            x=width
        )
        episode[f'robot0_gripper_width'] = gripper_interpolator(timestamps)
        # print(episode[f'robot0_gripper_width'])

        # print

        return episode


def sorted_directory_by_creation_time(path):
    try:
        directories = [p for p in path.iterdir() if p.is_dir()]

        sorted_directories = sorted(directories, key=lambda p: os.path.getmtime(p))

        return iter(sorted_directories)
    except FileNotFoundError:
        print(f" {directory} not exist")
        return iter([])


def main(args=None):

    input_dir = pathlib.Path('/root/cwh/umi_data/pick_medicine_sim_auto/samples')
    output_dir = pathlib.Path(os.path.expanduser('pick_medicine_sim_auto')).absolute()
    #output_dir = pathlib.Path('/home/meadow/cwh/ros2_ws/push_switch4')
    # hardcode subdirszarr_path = str(output_dir.joinpath('replay_buffer.zarr').absolute())
    replay_dir = output_dir.joinpath('replay_buffer.zarr')
    replay_buffer = ReplayBuffer.create_from_path(zarr_path=replay_dir, mode='a')
    video_dir = output_dir.joinpath('videos')
    video_dir.mkdir(exist_ok=True)
    episode_id = replay_buffer.n_episodes

    files = [f for f in input_dir.glob('pose_*') if f.is_file()]
    #print(files)
    files.sort(key=lambda f: f.stat().st_mtime)
    for  f in files:
        print(f)
        episode = csv2eposide(f)
        v = f.with_name(f.name.replace('pose_', 'video_').replace('.csv', '.mp4'))

        replay_buffer.add_episode(episode, compressors='disk')
        episode_id = replay_buffer.n_episodes - 1
        print(episode_id)
        vp = video_dir.joinpath(str(episode_id))
        vp.mkdir(exist_ok=True)
        shutil.copy(v, vp.joinpath('0.mp4'))


if __name__ == '__main__':
    main()
