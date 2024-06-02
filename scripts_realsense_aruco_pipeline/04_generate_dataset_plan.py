"""
python scripts_slam_pipeline/06_generate_dataset_plan.py -i data_workspace/cup_in_the_wild/20240105_zhenjia_packard_2nd_conference_room
"""

# %%
import sys
import os

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
from umi.common.cv_util import (
    get_gripper_width
)
from umi.common.interpolation_util import (
    get_gripper_calibration_interpolator, 
    get_interp1d
)

# %%
def get_bool_segments(bool_seq):
    bool_seq = np.array(bool_seq, dtype=bool)
    segment_ends = (np.nonzero(np.diff(bool_seq))[0] + 1).tolist()
    segment_bounds = [0] + segment_ends + [len(bool_seq)]
    segments = list()
    segment_type = list()
    for i in range(len(segment_bounds) - 1):
        start = segment_bounds[i]
        end = segment_bounds[i+1]
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
@click.option('-nz', '--nominal_z', type=float, default=0.072, help="nominal Z value for gripper finger tag")
@click.option('-ml', '--min_episode_length', type=int, default=24)
@click.option('--ignore_cameras', type=str, default=None, help="comma separated string of camera serials to ignore")
def main(input, output, tcp_offset, tx_slam_tag,
         nominal_z, min_episode_length, ignore_cameras):
    
    # param that should be agjusted
    # all unit in meters
    # y axis in camera frame
    cam_to_center_height = 0.086 # constant for UMI, change it to your setting
    # optical center to mounting screw, positive is when optical center is in front of the mount
    cam_to_mount_offset = 0.01465 # constant for GoPro Hero 9,10,11, change it to your setting

    # %% stage 0
    # gather inputs
    input_path = pathlib.Path(os.path.expanduser(input)).absolute()
    demos_dir = input_path.joinpath('demos')
    if output is None:
        output = input_path.joinpath('dataset_plan.pkl')

    # tcp to camera transform
    cam_to_tip_offset = cam_to_mount_offset + tcp_offset
    pose_cam_tcp = np.array([0, cam_to_center_height, cam_to_tip_offset, 0,0,0])
    tx_cam_tcp = pose_to_mat(pose_cam_tcp)
    
    # 对于aruco的场景，可以在环境中贴一个公共的二维码，当作世界坐标系，并且相机的轨迹是相对于该标记物的
    # 但是猜测跟这个坐标系关系不大，因为在训练的时候，使用的都是相对坐标系，可以搞个单位矩阵或者把代码删掉
    # 原始的slam方案中，会记录slam地图的原点相对于桌面二维码的变换
    # 在aruco的方案中，可以将该数值设置为外置相机相对于桌面某一个二维码的位置，下面的代码逻辑得以保留
    if tx_slam_tag is None:
        path = demos_dir.joinpath('mapping', 'tx_slam_tag.json')
        assert path.is_file()
        tx_slam_tag = str(path)
    tx_slam_tag = np.array(json.load(
        open(os.path.expanduser(tx_slam_tag), 'r')
        )['tx_slam_tag']
    )
    tx_tag_slam = np.linalg.inv(tx_slam_tag)

    # load gripper calibration
    gripper_id_gripper_cal_map = dict()
    cam_serial_gripper_cal_map = dict()

    # record cam_serial
    cam_serial = None
    with ExifToolHelper() as et:
        for gripper_cal_path in demos_dir.glob("gripper*/gripper_range.json"):
            mp4_path = gripper_cal_path.parent.joinpath('raw_video.mp4')
            if cam_serial is None:
                meta = list(et.get_metadata(str(mp4_path)))[0]
                cam_serial = meta['QuickTime:CameraSerialNumber']
            # TODO: realsense的相机中可能没有这个meta关键字，需要根据具体的视频进行修改

            gripper_range_data = json.load(gripper_cal_path.open('r'))
            gripper_id = gripper_range_data['gripper_id']
            max_width = gripper_range_data['max_width']
            min_width = gripper_range_data['min_width']
            gripper_cal_data = {
                'aruco_measured_width': [min_width, max_width],
                'aruco_actual_width': [min_width, max_width]
            }
            gripper_cal_interp = get_gripper_calibration_interpolator(**gripper_cal_data)
            gripper_id_gripper_cal_map[gripper_id] = gripper_cal_interp
            cam_serial_gripper_cal_map[cam_serial] = gripper_cal_interp

    # %% stage 1
    # loop over all demo directory to extract video metadata
    # output: video_meta_df
    
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
            # start_timestamp = 1704855454.882133     # read from the video

            if cam_serial in ignore_cam_serials:
                print(f"Ignored {video_dir.name}")
                continue
            
            csv_path = video_dir.joinpath('camera_trajectory.csv')
            if not csv_path.is_file():
                print(f"Ignored {video_dir.name}, no camera_trajectory.csv")
                continue
            
            pkl_path = video_dir.joinpath('tag_detection.pkl')
            if not pkl_path.is_file():
                print(f"Ignored {video_dir.name}, no tag_detection.pkl")
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

            # load SLAM data
            csv_path = video_dir.joinpath('camera_trajectory.csv')
            if not csv_path.is_file():
                print(f"Skipping {video_dir.name}, no camera_trajectory.csv.")
                dropped_camera_count[row['camera_serial']] += 1
                continue            
            
            csv_df = pd.read_csv(csv_path)
            # select aligned frames
            df = csv_df.iloc[start_frame_idx: start_frame_idx+n_frames]
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
            cam_pose[:,3,3] = 1
            cam_pose[:,:3,3] = cam_pos
            cam_pose[:,:3,:3] = cam_rot.as_matrix()
            tx_slam_cam = cam_pose
            tx_tag_cam = tx_tag_slam @ tx_slam_cam

            # TODO: handle optinal robot cal based filtering
            is_step_valid = is_tracked.copy()
            
            # get gripper data
            pkl_path = video_dir.joinpath('tag_detection.pkl')
            if not pkl_path.is_file():
                print(f"Skipping {video_dir.name}, no tag_detection.pkl.")
                dropped_camera_count[row['camera_serial']] += 1
                continue
                        
            tag_detection_results = pickle.load(open(pkl_path, 'rb'))
            # select aligned frames
            tag_detection_results = tag_detection_results[start_frame_idx: start_frame_idx+n_frames]

            # one item per frame
            video_timestamps = np.array([x['time'] for x in tag_detection_results])

            if len(df) != len(video_timestamps):
                print(f"Skipping {video_dir.name}, video csv length mismatch.")
                continue

            # get gripper action
            # TODO 1
            # ghi = row['gripper_hardware_id']
            ghi = 0
            if ghi < 0:
                print(f"Skipping {video_dir.name}, invalid gripper hardware id {ghi}")
                dropped_camera_count[row['camera_serial']] += 1
                continue
            
            left_id = 6 * ghi
            right_id = left_id + 1

            gripper_cal_interp = None
            if ghi in gripper_id_gripper_cal_map:
                gripper_cal_interp = gripper_id_gripper_cal_map[ghi]
            elif row['camera_serial'] in cam_serial_gripper_cal_map:
                gripper_cal_interp = cam_serial_gripper_cal_map[row['camera_serial']]
                print(f"Gripper id {ghi} not found in gripper calibrations {list(gripper_id_gripper_cal_map.keys())}. Falling back to camera serial map.")
            else:
                raise RuntimeError("Gripper calibration not found.")

            gripper_timestamps = list()
            gripper_widths = list()
            for td in tag_detection_results:
                width = get_gripper_width(td['tag_dict'], 
                    left_id=left_id, right_id=right_id, 
                    nominal_z=nominal_z)
                if width is not None:
                    gripper_timestamps.append(td['time'])
                    gripper_widths.append(gripper_cal_interp(width))
            gripper_interp = get_interp1d(gripper_timestamps, gripper_widths)
            
            gripper_det_ratio = (len(gripper_widths) / len(tag_detection_results))
            if gripper_det_ratio < 0.9:
                print(f"Warining: {video_dir.name} only {gripper_det_ratio} of gripper tags detected.")
            
            this_gripper_widths = gripper_interp(video_timestamps)
            
            # transform to tcp frame
            tx_tag_tcp = tx_tag_cam @ tx_cam_tcp
            pose_tag_tcp = mat_to_pose(tx_tag_tcp)
            
            # output value
            assert len(pose_tag_tcp) == n_frames
            assert len(this_gripper_widths) == n_frames
            assert len(is_step_valid) == n_frames
            all_cam_poses.append(pose_tag_tcp)
            all_gripper_widths.append(this_gripper_widths)
            all_is_valid.append(is_step_valid)

## %%
if __name__ == "__main__":
    main()