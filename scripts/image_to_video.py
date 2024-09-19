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
    
def arrangeAvailableRecordingsToNewDir(recording_dir):
    # Define the new directory path
    succeded_path = os.path.join(recording_dir, 'succeded_data')
    failed_path = os.path.join(recording_dir, 'failed_data')

    # Create the new directory if it doesn't exist
    if not os.path.exists(succeded_path):
        os.makedirs(succeded_path)
        
    if not os.path.exists(failed_path):
        os.makedirs(failed_path)

    # Iterate over all directories in the recording directory
    for folder_name in os.listdir(recording_dir):
        folder_path = os.path.join(recording_dir, folder_name)
        
        # Check if the folder name starts with 'umi_' and it is a directory
        if os.path.isdir(folder_path) and folder_name.startswith('umi_'):
            # Extract the date and time from the folder name
            date_time = folder_name.split('_', 1)[1]

            # Define the paths to the files and directories inside the umi folder
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

if __name__ == "__main__":
    recording_folder = '/home/robot/Dev/test/recording'  

    for folder_name in os.listdir(recording_folder):
        folder_path = os.path.join(recording_folder, folder_name)
        if os.path.isdir(folder_path):
            pictures_folder = os.path.join(folder_path, 'pictures')
            if os.path.exists(pictures_folder):
                process_recording_folder(folder_path)
    
    arrangeAvailableRecordingsToNewDir(recording_folder)
