import os
import cv2
import re
import csv

def extract_image_index(image_name):
    # 使用正则表达式提取图片名称中的序号
    match = re.search(r'rgb_(\d+)_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.png', image_name)
    if match:
        return int(match.group(1))
    else:
        return -1 

def extract_timestamps_from_txt(txt_file):
    timestamps = {}
    with open(txt_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            parts = line.strip().split(',')
            frame_idx = int(parts[0].split(':')[1])
            timestamp = float(parts[1].split(':')[1])
            width = float(parts[2].split(':')[1])  
            timestamps[frame_idx] = (timestamp, width)
    return timestamps

def extract_timestamps_from_csv(csv_file):
    timestamps = {}
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Read header
        idx_map = {header[i]: i for i in range(len(header))}
        for row in reader:
            frame_idx = int(row[0])
            timestamp = float(row[1])
            other_data = row[2:]  
            timestamps[frame_idx] = (timestamp, other_data)
    return timestamps

def process_recording_folder(recording_folder, target_fps=30):
    print(f"Processing folder: {recording_folder}")
    pictures_folder = os.path.join(recording_folder, 'pictures')
    jaw_width_file = os.path.join(recording_folder, 'jaw_width.txt')
    position_file = os.path.join(recording_folder, 'position.csv')

    # Step 1: Generate video with uniform frame rate
    fps = target_fps
    output_video_file = os.path.join(recording_folder, 'raw_video.mp4')
    images_to_video(pictures_folder, output_video_file, fps)

    # Step 2: Modify timestamps in txt and csv files
    timestamps_from_txt = extract_timestamps_from_txt(jaw_width_file)
    timestamps_from_csv = extract_timestamps_from_csv(position_file)

    # Modify timestamps
    dt = 1.0 / target_fps
    new_timestamps_txt = {}
    new_timestamps_csv = {}

    # Update timestamps for txt file
    for frame_idx in sorted(timestamps_from_txt.keys()):
        if frame_idx == 0:
            new_timestamps_txt[frame_idx] = (0.0, timestamps_from_txt[frame_idx][1]) 
        else:
            new_timestamps_txt[frame_idx] = (dt * frame_idx, timestamps_from_txt[frame_idx][1]) 

    # Update timestamps for csv file
    for frame_idx in sorted(timestamps_from_csv.keys()):
        if frame_idx == 0:
            new_timestamps_csv[frame_idx] = (0.0, timestamps_from_csv[frame_idx][1])
        else:
            new_timestamps_csv[frame_idx] = (dt * frame_idx, timestamps_from_csv[frame_idx][1])

    # Write back updated timestamps to txt file
    with open(jaw_width_file, 'w') as f:
        for frame_idx in sorted(new_timestamps_txt.keys()):
            timestamp, width = new_timestamps_txt[frame_idx]
            f.write(f"frame_idx:{frame_idx}, timestamp:{timestamp:.6f} width:{width:.6f}\n")

    # Write back updated timestamps to csv file
    with open(position_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_idx', 'timestamp', 'state', 'is_lost', 'is_keyframe', 'x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w'])
        for frame_idx in sorted(new_timestamps_csv.keys()):
            timestamp, other_data = new_timestamps_csv[frame_idx]
            row = [frame_idx, timestamp] + other_data
            writer.writerow(row)

def images_to_video(image_folder, output_video_file, fps):
    # 获取所有图片文件名并按照序号排序
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images_sorted = sorted(images, key=lambda x: extract_image_index(x))

    first_image_path = os.path.join(image_folder, images_sorted[0])
    first_image = cv2.imread(first_image_path)
    height, width, layers = first_image.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video_file, fourcc, fps, (width, height))

    for image in images_sorted:
        image_path = os.path.join(image_folder, image)
        frame = cv2.imread(image_path)
        video.write(frame)

    video.release()

if __name__ == "__main__":
    recording_folder = '/home/robot/Dev/test/recording'  

    for folder_name in os.listdir(recording_folder):
        folder_path = os.path.join(recording_folder, folder_name)
        if os.path.isdir(folder_path):
            pictures_folder = os.path.join(folder_path, 'pictures')
            if os.path.exists(pictures_folder):
                process_recording_folder(folder_path)
