import os.path
import sys
import glob
import subprocess

meshdir = sys.argv
print(len(meshdir))
if len(meshdir) != 2:
    print("$ python convert.py meshdir")
    sys.exit()

meshdir = meshdir[1]

def run_subprocess(cmd):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True)


def process_subdirectories(root):
    for foldername, subfolders, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith(".dae"):
                input_file = os.path.join(foldername, filename)
                output_file = os.path.join(foldername, os.path.splitext(filename)[0] + ".stl")

                script_path = os.path.abspath(__file__)
                script_dir = os.path.dirname(script_path)

                cmd = "xvfb-run -a meshlabserver -i {} -o {} -m binary".format(input_file, output_file)
                run_subprocess(cmd)

process_subdirectories(meshdir)
