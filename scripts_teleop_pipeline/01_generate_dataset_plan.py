"""
python scripts_slam_pipeline/06_generate_dataset_plan.py -i data_workspace/cup_in_the_wild/20240105_zhenjia_packard_2nd_conference_room
"""

# %%
import sys
import os
from datetime import datetime

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib
import click
import pickle
import numpy as np
import json
import math
import collections
import scipy.ndimage as sn
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import av
from exiftool import ExifToolHelper
from umi.common.timecode_util import mp4_get_start_datetime
from umi.common.pose_util import pose_to_mat, mat_to_pose


# %%
def get_bool_segments(bool_seq):
    bool_seq = np.array(bool_seq, dtype=bool)
    segment_ends = (np.nonzero(np.diff(bool_seq))[0] + 1).tolist()
    segment_bounds = [0] + segment_ends + [len(bool_seq)]
    segments = list()
    segment_type = list()
    for i in range(len(segment_bounds) - 1):
        start = segment_bounds[i]
        end = segment_bounds[i + 1]
        this_type = bool_seq[start]
        segments.append(slice(start, end))
        segment_type.append(this_type)
    segment_type = np.array(segment_type, dtype=bool)
    return segments, segment_type


# %%
@click.command()
@click.option('-i', '--input', required=True, help='Project directory')
@click.option('-o', '--output', default=None)
@click.option('-to', '--tcp_offset', type=float, default=0.205, help="Distance from gripper tip to mounting screw")
@click.option('-ts', '--tx_slam_tag', default=None, help="tx_slam_tag.json")
@click.option('-nz', '--nominal_z', type=float, default=0.31, help="nominal Z value for gripper finger tag")
@click.option('-ml', '--min_episode_length', type=int, default=24)
@click.option('--ignore_cameras', type=str, default=None, help="comma separated string of camera serials to ignore")
def main(input, output, tcp_offset, tx_slam_tag,
         nominal_z, min_episode_length, ignore_cameras):

    # %% stage 0
    # gather inputs
    input_path = pathlib.Path(os.path.expanduser(input)).absolute()
    demos_dir = input_path.joinpath('demos')
    if output is None:
        output = input_path.joinpath('dataset_plan.pkl')

    # %% stage 1
    # loop over all demo directory to extract video metadata
    # output: video_meta_df
    cam_serial = '00001'
    # find videos
    video_dirs = sorted([x.parent for x in demos_dir.glob('demo_*/raw_video.mp4')])

    # ignore camera，在多相机模式下，可以指定忽略的相机
    ignore_cam_serials = set()
    if ignore_cameras is not None:
        serials = ignore_cameras.split(',')
        ignore_cam_serials = set(serials)

    fps = None
    rows = list()
    with ExifToolHelper() as et:
        for video_dir in video_dirs:
            mp4_path = video_dir.joinpath('raw_video.mp4')
            # TODO：这个在存储视频的时候需要保存，函数的实现需要修改
            start_date = mp4_get_start_datetime(str(mp4_path))
            start_timestamp = start_date.timestamp()

            if cam_serial in ignore_cam_serials:
                print(f"Ignored {video_dir.name}")
                continue

            csv_path = video_dir.joinpath('camera_trajectory.csv')
            if not csv_path.is_file():
                print(f"Ignored {video_dir.name}, no camera_trajectory.csv")
                continue

            pkl_path = video_dir.joinpath('gripper_width.csv')
            if not pkl_path.is_file():
                print(f"Ignored {video_dir.name}, no gripper_width.csv")
                continue

            with av.open(str(mp4_path), 'r') as container:
                stream = container.streams.video[0]
                n_frames = stream.frames
                if fps is None:
                    fps = stream.average_rate
                else:
                    if fps != stream.average_rate:
                        print(f"Inconsistent fps: {float(fps)} vs {float(stream.average_rate)} in {video_dir.name}")
                        exit(1)
            duration_sec = float(n_frames / fps)
            end_timestamp = start_timestamp + duration_sec

            rows.append({
                'video_dir': video_dir,
                'camera_serial': cam_serial,
                'n_frames': n_frames,
                'fps': fps,
                'start_timestamp': start_timestamp,
                'end_timestamp': end_timestamp
            })
    if len(rows) == 0:
        print("No valid videos found!")
        exit(1)
    # print(rows)
    video_meta_df = pd.DataFrame(data=rows)

    # %% stage 2
    # match videos into demos
    # output:
    # demo_data_list = {
    #     "video_idxs": [int],
    #     # calculating start/end frame requires gripper info, defer to later stage
    #     "start_timestamp": float,
    #     "end_timestamp": float
    # }
    # map serial to count
    serial_count = video_meta_df['camera_serial'].value_counts()
    print("Found following cameras:")
    print(serial_count)
    n_cameras = len(serial_count)

    events = list()
    for vid_idx, row in video_meta_df.iterrows():
        events.append({
            'vid_idx': vid_idx,
            'camera_serial': row['camera_serial'],
            't': row['start_timestamp'],
            'is_start': True
        })
        events.append({
            'vid_idx': vid_idx,
            'camera_serial': row['camera_serial'],
            't': row['end_timestamp'],
            'is_start': False
        })
    events = sorted(events, key=lambda x: x['t'])

    demo_data_list = list()
    on_videos = set()
    on_cameras = set()
    used_videos = set()
    t_demo_start = None
    # print(events)
    for i, event in enumerate(events):
        # update state based on event
        if event['is_start']:
            on_videos.add(event['vid_idx'])
            on_cameras.add(event['camera_serial'])
        else:
            on_videos.remove(event['vid_idx'])
            on_cameras.remove(event['camera_serial'])
        assert len(on_videos) == len(on_cameras)

        if len(on_cameras) == n_cameras:
            # start demo episode where all cameras are recording
            t_demo_start = event['t']
        elif t_demo_start is not None:
            # demo already started, but one camera stopped
            # stopping episode
            assert not event['is_start']

            t_start = t_demo_start
            t_end = event['t']

            # undo state update to get full set of videos
            demo_vid_idxs = set(on_videos)
            demo_vid_idxs.add(event['vid_idx'])
            used_videos.update(demo_vid_idxs)

            demo_data_list.append({
                "video_idxs": sorted(demo_vid_idxs),
                "start_timestamp": t_start,
                "end_timestamp": t_end
            })
            t_demo_start = None
    unused_videos = set(video_meta_df.index) - used_videos
    for vid_idx in unused_videos:
        print(f"Warning: video {video_meta_df.loc[vid_idx]['video_dir'].name} unused in any demo")

    # %% stage 3
    # generate dataset plan
    # output
    # all_plans = [{
    #     "episode_timestamps": np.ndarray,
    #     "grippers": [{
    #         "tcp_pose": np.ndarray,
    #         "gripper_width": np.ndarray
    #     }],
    #     "cameras": [{
    #         "video_path": str,
    #         "video_start_end": Tuple[int,int]
    #     }]
    # }]
    # 假设系统中只有一个相机
    n_gripper_cams = 1

    cam_serial_cam_idx_map = dict()
    cam_serial_cam_idx_map[cam_serial] = 0  # 如果有多个相机，可能需要修改
    camera_idx_series = video_meta_df['camera_serial'].map(cam_serial_cam_idx_map)
    video_meta_df['camera_idx'] = camera_idx_series

    total_avaliable_time = 0.0
    total_used_time = 0.0
    dropped_camera_count = collections.defaultdict(lambda: 0)
    n_dropped_demos = 0
    all_plans = list()
    print(f"demo_data_list: {demo_data_list}")
    for demo_idx, demo_data in enumerate(demo_data_list):
        video_idxs = demo_data['video_idxs']
        start_timestamp = demo_data['start_timestamp']
        end_timestamp = demo_data['end_timestamp']
        total_avaliable_time += (end_timestamp - start_timestamp)

        # select relevant video data
        demo_video_meta_df = video_meta_df.loc[video_idxs].copy()
        # TODO 1
        demo_video_meta_df.set_index('camera_idx', inplace=True)
        demo_video_meta_df.sort_index(inplace=True)

        # determine optimal alignment
        dt = None
        alignment_costs = list()
        for cam_idx, row in demo_video_meta_df.iterrows():
            dt = 1 / row['fps']
            this_alignment_cost = list()
            for other_cam_idx, other_row in demo_video_meta_df.iterrows():
                # what's the delay for previous frame
                diff = other_row['start_timestamp'] - row['start_timestamp']
                remainder = diff % dt
                this_alignment_cost.append(remainder)
            alignment_costs.append(this_alignment_cost)
        # first video in bundle
        align_cam_idx = np.argmin([sum(x) for x in alignment_costs])

        # rewrite start_timestamp to be integer multiple of dt
        align_video_start = demo_video_meta_df.loc[align_cam_idx]['start_timestamp']
        start_timestamp += dt - ((start_timestamp - align_video_start) % dt)

        print(
            f"start_timestamp: {start_timestamp}, end_timestamp: {end_timestamp}, dt: {int((end_timestamp - start_timestamp) / dt)}")

        # descritize timestamps for all videos
        cam_start_frame_idxs = list()
        n_frames = int((end_timestamp - start_timestamp) / dt)
        for cam_idx, row in demo_video_meta_df.iterrows():
            video_start_frame = math.ceil((start_timestamp - row['start_timestamp']) / dt)
            video_n_frames = math.floor((row['end_timestamp'] - start_timestamp) / dt) - 1
            if video_start_frame < 0:
                video_n_frames += video_start_frame
                video_start_frame = 0
            cam_start_frame_idxs.append(video_start_frame)
            n_frames = min(n_frames, video_n_frames)
        demo_timestamps = np.arange(n_frames) * float(dt) + start_timestamp
        # print(demo_timestamps)

        # load pose and gripper data for each video
        # determin valid frames for each video
        all_cam_poses = list()
        all_gripper_widths = list()
        all_is_valid = list()

        for cam_idx, row in demo_video_meta_df.iterrows():
            if cam_idx >= n_gripper_cams:
                # not gripper camera
                continue

            start_frame_idx = cam_start_frame_idxs[cam_idx]
            video_dir = row['video_dir']
            print(f"start_frame_idx: {start_frame_idx}")
            # load SLAM data
            csv_path = video_dir.joinpath('camera_trajectory.csv')
            if not csv_path.is_file():
                print(f"Skipping {video_dir.name}, no camera_trajectory.csv.")
                dropped_camera_count[row['camera_serial']] += 1
                continue

            csv_df = pd.read_csv(csv_path)
            csv_df.set_index('frame_idx',inplace=True)
            # print(csv_df.iloc[0])
            csv_df = csv_df.loc[~csv_df.index.duplicated(), :]
            full_index = pd.RangeIndex(start=0, stop=csv_df.index.max()+1, step=1)
            # csv_df = csv_df[~csv_df.index.duplicated(keep='first')]
            df_reindexed = csv_df.reindex(full_index)
            df_reindexed.iloc[0] = df_reindexed.iloc[1]
            df_reindexed.iloc[0]['timestamp'] = 0
            df_reindexed.iloc[0]['is_lost'] = True
            df_reindexed["timestamp"] = df_reindexed["timestamp"].fillna(0)
            df_reindexed["is_lost"] = df_reindexed["is_lost"].fillna(True)
            df_reindexed["x"] = df_reindexed["x"].fillna(0)
            df_reindexed["y"] = df_reindexed["y"].fillna(0)
            df_reindexed["z"] = df_reindexed["z"].fillna(0)
            df_reindexed["q_x"] = df_reindexed["q_x"].fillna(0.0)
            df_reindexed["q_y"] = df_reindexed["q_y"].fillna(0.0)
            df_reindexed["q_z"] = df_reindexed["q_z"].fillna(0.0)
            df_reindexed["q_w"] = df_reindexed["q_w"].fillna(0.0)

            # select aligned frames
            df = df_reindexed.iloc[start_frame_idx: start_frame_idx + n_frames]

            is_tracked = (~df['is_lost']).to_numpy()

            # basic filtering to remove bad tracking
            n_frames_lost = (~is_tracked).sum()
            if n_frames_lost > 10:
                print(f"Skipping {video_dir.name}, {n_frames_lost} frames are lost.")
                dropped_camera_count[row['camera_serial']] += 1
                continue

            n_frames_valid = is_tracked.sum()
            if n_frames_valid < 60:
                print(f"Skipping {video_dir.name}, only {n_frames_valid} frames are valid.")
                dropped_camera_count[row['camera_serial']] += 1
                continue

            # load camera pose
            df.loc[df['is_lost'], 'q_w'] = 1
            cam_pos = df[['x', 'y', 'z']].to_numpy()
            cam_rot_quat_xyzw = df[['q_x', 'q_y', 'q_z', 'q_w']].to_numpy()
            cam_rot = Rotation.from_quat(cam_rot_quat_xyzw)
            cam_pose = np.zeros((cam_pos.shape[0], 4, 4), dtype=np.float32)
            cam_pose[:, 3, 3] = 1
            cam_pose[:, :3, 3] = cam_pos
            cam_pose[:, :3, :3] = cam_rot.as_matrix()
            # tx_slam_cam = cam_pose
            # tx_tag_cam = tx_tag_slam @ tx_slam_cam

            # TODO: handle optinal robot cal based filtering
            is_step_valid = is_tracked.copy()
            # get gripper data
            gripper_path = video_dir.joinpath('gripper_width.csv')
            if not gripper_path.is_file():
                print(f"Skipping {video_dir.name}, no gripper_width.csv.")
                dropped_camera_count[row['camera_serial']] += 1
                continue
            gripper_df = pd.read_csv(gripper_path)
            gripper_df.set_index('frame_idx', inplace=True)
            # print(gripper_df.loc[gripper_df.index.duplicated(), :])
            gripper_df = gripper_df.loc[~gripper_df.index.duplicated(), :]
            full_index = pd.RangeIndex(start=0, stop=gripper_df.index.max() + 1, step=1)
            df_reindexed = gripper_df.reindex(full_index)
            df_reindexed.iloc[0] = df_reindexed.iloc[1]
            df_reindexed.iloc[0]['timestamp'] = 0
            gripper_df = df_reindexed[start_frame_idx : start_frame_idx+n_frames]

            idx = start_frame_idx
            this_gripper_widths = gripper_df['width'].values
            print([  x for x in this_gripper_widths])
            print(f"nframe: {n_frames}")
            print(f"width: {this_gripper_widths}, len:{len(this_gripper_widths)},max: {gripper_df.shape}")
            pose_tag_tcp = mat_to_pose(cam_pose)
            print(f"pose_tag_tcp {pose_tag_tcp}, len: {len(pose_tag_tcp)}, max: {cam_pose.shape},")

            # output value
            assert len(pose_tag_tcp) == n_frames
            assert len(this_gripper_widths) == n_frames
            assert len(is_step_valid) == n_frames
            all_cam_poses.append(pose_tag_tcp)
            all_gripper_widths.append(this_gripper_widths)
            all_is_valid.append(is_step_valid)

        if len(all_cam_poses) != n_gripper_cams:
            print(f"Skipped demo {demo_idx}.")
            n_dropped_demos += 1
            continue

        # aggregate valid result
        all_is_valid = np.array(all_is_valid)
        is_step_valid = np.all(all_is_valid, axis=0)

        # generate episode start and end pose for each gripper
        first_valid_step = np.nonzero(is_step_valid)[0][0]
        last_valid_step = np.nonzero(is_step_valid)[0][-1]
        print(f"first_valid_step: {first_valid_step}\nlast_valid_step: {last_valid_step}")

        demo_start_poses = list()
        demo_end_poses = list()
        for cam_idx in range(len(all_cam_poses)):
            cam_poses = all_cam_poses[cam_idx]
            demo_start_poses.append(cam_poses[first_valid_step])
            demo_end_poses.append(cam_poses[last_valid_step])

        # determine episode segmentation
        # remove valid segments that are too short
        segment_slices, segment_type = get_bool_segments(is_step_valid)
        for s, is_valid_segment in zip(segment_slices, segment_type):
            start = s.start
            end = s.stop
            if not is_valid_segment:
                continue
            if (end - start) < min_episode_length:
                is_step_valid[start:end] = False

        # finally, generate one episode for each valid segment
        segment_slices, segment_type = get_bool_segments(is_step_valid)
        for s, is_valid in zip(segment_slices, segment_type):
            if not is_valid:
                continue
            start = s.start
            end = s.stop

            total_used_time += float((end - start) * dt)

            grippers = list()
            cameras = list()
            for cam_idx, row in demo_video_meta_df.iterrows():
                if cam_idx < n_gripper_cams:
                    pose_tag_tcp = all_cam_poses[cam_idx][start:end]

                    # gripper cam
                    grippers.append({
                        "tcp_pose": pose_tag_tcp,
                        "gripper_width": all_gripper_widths[cam_idx][start:end],
                        "demo_start_pose": demo_start_poses[cam_idx],
                        "demo_end_pose": demo_end_poses[cam_idx]
                    })
                # all cams
                video_dir = row['video_dir']
                vid_start_frame = cam_start_frame_idxs[cam_idx]
                cameras.append({
                    "video_path": str(video_dir.joinpath('raw_video.mp4').relative_to(video_dir.parent)),
                    "video_start_end": (start + vid_start_frame, end + vid_start_frame)
                })

            all_plans.append({
                "episode_timestamps": demo_timestamps[start:end],
                "grippers": grippers,
                "cameras": cameras
            })
            print([g['gripper_width'] for g in grippers])

    used_ratio = total_used_time / total_avaliable_time
    print(f"{int(used_ratio * 100)}% of raw data are used.")

    print(dropped_camera_count)
    print("n_dropped_demos", n_dropped_demos)

    # print(all_plans)
    # %%
    # dump the plan to pickle
    pickle.dump(all_plans, output.open('wb'))


## %%
if __name__ == "__main__":
    main()
