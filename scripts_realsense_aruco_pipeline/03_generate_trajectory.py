"""
python scripts_realsense_aruco_pipeline/03_generate_trajectory.py ../example_demo_session
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
    pass
            
# %%
if __name__ == "__main__":
    main()