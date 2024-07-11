import os
import subprocess
from common import get_directory_names, HOST_ROS_SRC

def git_pull():
    old_wd = os.getcwd()
    directories = get_directory_names(HOST_ROS_SRC)
    for dirname in directories:
        if dirname.startswith('road_quality_') or dirname == 'sync_rpi4':
            os.chdir(os.path.join(HOST_ROS_SRC, dirname))
            print(f'========= {dirname} =========')
            stdout = subprocess.check_output(["git", "pull"]).decode().strip()
            print(stdout)
    os.chdir(old_wd)


if __name__ == '__main__':
    git_pull()
