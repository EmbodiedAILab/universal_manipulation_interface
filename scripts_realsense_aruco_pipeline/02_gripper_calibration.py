"""
python scripts_realsense_aruco_pipeline/02_gripper_calibration.py ../example_demo_session
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
import subprocess

# %%
@click.command()
@click.argument('session_dir', nargs=-1)
def main(session_dir):
    script_dir = pathlib.Path(__file__).parent.parent.joinpath('scripts')
    
    for session in session_dir:
        session = pathlib.Path(session)
        demos_dir = session.joinpath('demos')
        
        # run gripper range calibration
        script_path = script_dir.joinpath('calibrate_gripper_range.py')
        assert script_path.is_file()
        
        for gripper_dir in demos_dir.glob("gripper_calibration*"):
            gripper_range_path = gripper_dir.joinpath('gripper_range.json')
            tag_path = gripper_dir.joinpath('tag_detection.pkl')
            assert tag_path.is_file()
            cmd = [
                'python', str(script_path),
                '--input', str(tag_path),
                '--output', str(gripper_range_path)
            ]
            subprocess.run(cmd)

            
# %%
if __name__ == "__main__":
    main()
