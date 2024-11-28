"""
Main script for UMI Realsense Aruco pipeline.
python run_realsense_aruco_pipeline.py <session_dir>
"""

import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib
import click
import subprocess

# %%
@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('-c', '--calibration_dir', type=str, default=None)
def main(session_dir, calibration_dir):
    script_dir = pathlib.Path(__file__).parent
    # script_dir = pathlib.Path(__file__).parent.joinpath('scripts_realsense_aruco_pipeline')
    # if calibration_dir is None:
    #     calibration_dir = pathlib.Path(__file__).parent.joinpath('example', 'calibration')
    # else:
    #     calibration_dir = pathlib.Path(calibration_dir)
    # assert calibration_dir.is_dir()

    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        print(session)

        print("############## 00_process_videos #############")
        script_path = script_dir.joinpath("00_process_videos.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

        print("############# 02_generate_dataset_plan ###########")
        script_path = script_dir.joinpath("01_generate_dataset_plan.py")
        assert script_path.is_file()
        cmd = [
            'python', str(script_path),
            '--input', str(session)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

## %%
if __name__ == "__main__":
    main()
