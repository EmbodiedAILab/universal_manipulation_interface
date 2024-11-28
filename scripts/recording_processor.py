import os
import cv2
import re
import csv
import shutil
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# 初始化成功和失败计数
success_count = 0
failure_count = 0

def extract_image_index(image_name):
    match = re.search(r'rgb_(\d+)_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.png', image_name)
    if match:
        return int(match.group(1))
    else:
        return -1

def extract_timestamps_from_csv(csv_file):
    timestamps = {}
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Read header
        for row in reader:
            frame_idx = int(row[0])
            timestamp = float(row[1])
            data = row[2:]
            timestamps[frame_idx] = (timestamp, data)
    return timestamps

def process_recording_folder(recording_folder, target_fps=30):
    print(f"Processing folder: {recording_folder}")
    pictures_folder = os.path.join(recording_folder, 'pictures')
    jaw_width_file = os.path.join(recording_folder, 'jaw_width.csv')
    position_file = os.path.join(recording_folder, 'position.csv')

    fps = target_fps
    output_video_file = os.path.join(recording_folder, 'raw_video.mp4')
    images_to_video(pictures_folder, output_video_file, fps)

    timestamps_from_jaw_width = extract_timestamps_from_csv(jaw_width_file)
    timestamps_from_position = extract_timestamps_from_csv(position_file)

    dt = 1.0 / target_fps
    new_timestamps_jaw_width = {}
    new_timestamps_position = {}

    for frame_idx in sorted(timestamps_from_jaw_width.keys()):
        if frame_idx == 0:
            new_timestamps_jaw_width[frame_idx] = (0.0, timestamps_from_jaw_width[frame_idx][1])
        else:
            new_timestamps_jaw_width[frame_idx] = (dt * frame_idx, timestamps_from_jaw_width[frame_idx][1])

    for frame_idx in sorted(timestamps_from_position.keys()):
        if frame_idx == 0:
            new_timestamps_position[frame_idx] = (0.0, timestamps_from_position[frame_idx][1])
        else:
            new_timestamps_position[frame_idx] = (dt * frame_idx, timestamps_from_position[frame_idx][1])

    with open(jaw_width_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_idx', 'timestamp', 'width'])
        for frame_idx in sorted(new_timestamps_jaw_width.keys()):
            timestamp, width = new_timestamps_jaw_width[frame_idx]
            writer.writerow([frame_idx, timestamp, width[0]])

    with open(position_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_idx', 'timestamp', 'state', 'is_lost', 'is_keyframe', 'x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w'])
        for frame_idx in sorted(new_timestamps_position.keys()):
            timestamp, other_data = new_timestamps_position[frame_idx]
            row = [frame_idx, timestamp] + other_data
            writer.writerow(row)

def images_to_video(image_folder, output_video_file, fps):
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

def arrangeAvailableRecordingsToNewDir(folder_path):
    global success_count, failure_count
    
    # Define the new directory path
    recording_dir = os.path.dirname(folder_path)
    succeded_path = os.path.join(recording_dir, 'succeded_data')
    failed_path = os.path.join(recording_dir, 'failed_data')

    # Create the new directory if it doesn't exist
    if not os.path.exists(succeded_path):
        os.makedirs(succeded_path)
        
    if not os.path.exists(failed_path):
        os.makedirs(failed_path)

    folder_name = os.path.basename(folder_path)
    date_time = folder_name.split('_', 1)[1]

    pictures_path = os.path.join(folder_path, 'pictures')
    jaw_width_path = os.path.join(folder_path, 'jaw_width.csv')
    position_path = os.path.join(folder_path, 'position.csv')
    raw_video_path = os.path.join(folder_path, 'raw_video.mp4')
    true_path = os.path.join(folder_path, 'true')
    false_path = os.path.join(folder_path, 'false')
    
    # Remove the pictures directory and its contents
    if os.path.exists(pictures_path) and os.path.exists(raw_video_path):
        shutil.rmtree(pictures_path)
        print(f'Deleted {pictures_path}')
    
    if os.path.exists(true_path):
        # Rename and move jaw_width.csv
        if os.path.exists(jaw_width_path):
            new_jaw_width_name = f'gripper_{date_time}.csv'
            new_jaw_width_path = os.path.join(succeded_path, new_jaw_width_name)
            shutil.move(jaw_width_path, new_jaw_width_path)
            print(f'Moved {jaw_width_path} to {new_jaw_width_path}')
        
        # Rename and move position.csv
        if os.path.exists(position_path):
            new_position_name = f'pose_{date_time}.csv'
            new_position_path = os.path.join(succeded_path, new_position_name)
            shutil.move(position_path, new_position_path)
            print(f'Moved {position_path} to {new_position_path}')
        
        # Rename and move raw_video.mp4
        if os.path.exists(raw_video_path):
            new_video_name = f'video_{date_time}.mp4'
            new_video_path = os.path.join(succeded_path, new_video_name)
            shutil.move(raw_video_path, new_video_path)
            print(f'Moved {raw_video_path} to {new_video_path}')
        
        # 成功计数加一
        success_count += 1

    else:
        if os.path.exists(jaw_width_path):
            new_jaw_width_name = f'gripper_{date_time}.csv'
            new_jaw_width_path = os.path.join(failed_path, new_jaw_width_name)
            shutil.move(jaw_width_path, new_jaw_width_path)
            print(f'Moved {jaw_width_path} to {new_jaw_width_path}')
        
        # Rename and move position.csv
        if os.path.exists(position_path):
            new_position_name = f'pose_{date_time}.csv'
            new_position_path = os.path.join(failed_path, new_position_name)
            shutil.move(position_path, new_position_path)
            print(f'Moved {position_path} to {new_position_path}')
        
        # Rename and move raw_video.mp4
        if os.path.exists(raw_video_path):
            new_video_name = f'video_{date_time}.mp4'
            new_video_path = os.path.join(failed_path, new_video_name)
            shutil.move(raw_video_path, new_video_path)
            print(f'Moved {raw_video_path} to {new_video_path}')
        
        # 失败计数加一
        failure_count += 1

    # 打印当前成功和失败的个数
    print(f"======= Success count: {success_count}")
    print(f"======= Failure count: {failure_count}")

    # Delete the folder and its contents
    shutil.rmtree(folder_path)
    print(f'Deleted folder {folder_path}')

class MyHandler(FileSystemEventHandler):
    def __init__(self, recording_dir):
        self.recording_dir = recording_dir
        self.watched_folders = set()
    
    def on_created(self, event):
        if event.is_directory:
            folder_name = os.path.basename(event.src_path)
            if folder_name.startswith('umi_'):
                self.watched_folders.add(event.src_path)
                print(f"Started watching {event.src_path}")

    def on_any_event(self, event):
        for folder in list(self.watched_folders):
            true_path = os.path.join(folder, 'true')
            false_path = os.path.join(folder, 'false')
            if os.path.exists(true_path) or os.path.exists(false_path):
                print(f"Processing {folder}")
                process_recording_folder(folder)
                arrangeAvailableRecordingsToNewDir(folder)
                self.watched_folders.remove(folder)
                print(f"Stopped watching {folder}")

if __name__ == "__main__":
    recording_folder = os.getenv('RECORDING_FOLDER')

    event_handler = MyHandler(recording_folder)
    observer = Observer()
    observer.schedule(event_handler, path=recording_folder, recursive=True)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
