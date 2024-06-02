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

## %%
if __name__ == "__main__":
    main()