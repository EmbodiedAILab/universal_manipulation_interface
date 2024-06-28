import os
import re
import subprocess
from datetime import datetime
import pytz


def extract_time(file_path):
    file_name = os.path.basename(file_path)
    time_pattern = r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})"
    match = re.search(time_pattern, file_name)
    if match:
        dt = match.group(0)
        # idx = dt.rfind('_')
        local_time = datetime.strptime(dt, '%Y-%m-%d_%H-%M-%S')
        local_tz = pytz.timezone('Asia/Shanghai')
        local_time = local_tz.localize(local_time)
        utc_time = local_time.astimezone(pytz.utc)
        return utc_time
    return None

def exec_command(dir_path,command):
    if not os.path.exists(dir_path):
        print("not exist")
        return

    for root, dirs, files in os.walk(dir_path):
        for file in files:
            if file.endswith(".mp4"):
                file_path = os.path.join(root, file)
                time = extract_time(file_path)
                command = ['ffmpeg', '-i', file_path, '-metadata', f'creation_time={time.strftime("%Y-%m-%dT%H:%M:%SZ")}','-timecode', f'{time.strftime("%H:%M:%S")}:00','-codec', 'copy', f'{root}/result/{file}']
                print(' '.join(command))
                subprocess.run(command)
                
                # 执行FFmpeg命令


dir_path="/home/robo/cwh/universal_manipulation_interface/data/collect/"

exec_command(dir_path, "")