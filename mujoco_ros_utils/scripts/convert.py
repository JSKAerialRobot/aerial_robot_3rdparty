import os.path
import sys
import glob
import subprocess

def run_subprocess(cmd):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True)

def convert_dae_to_stl(input_file, output_file):
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    filter_path = os.path.join(script_dir, "../config/filter.mxl")

    cmd = "xvfb-run -a meshlabserver -i {} -o {} -m binary -s {}".format(input_file, output_file, filter_path)
    run_subprocess(cmd)
