import os
import shutil

# Define the path to the recording directory
recording_dir = '/home/edgelab/recording'

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
        
        # Remove the pictures directory and its contents
        if os.path.exists(pictures_path):
            shutil.rmtree(pictures_path)
            print(f'Deleted {pictures_path}')
        
        # Rename and move jaw_width.csv
        if os.path.exists(jaw_width_path):
            new_jaw_width_name = f'gripper_{date_time}.csv'
            new_jaw_width_path = os.path.join(recording_dir, new_jaw_width_name)
            shutil.move(jaw_width_path, new_jaw_width_path)
            print(f'Moved {jaw_width_path} to {new_jaw_width_path}')
        
        # Rename and move position.csv
        if os.path.exists(position_path):
            new_position_name = f'pose_{date_time}.csv'
            new_position_path = os.path.join(recording_dir, new_position_name)
            shutil.move(position_path, new_position_path)
            print(f'Moved {position_path} to {new_position_path}')
        
        # Rename and move raw_video.mp4
        if os.path.exists(raw_video_path):
            new_video_name = f'video_{date_time}.mp4'
            new_video_path = os.path.join(recording_dir, new_video_name)
            shutil.move(raw_video_path, new_video_path)
            print(f'Moved {raw_video_path} to {new_video_path}')

print('Processing complete.')
