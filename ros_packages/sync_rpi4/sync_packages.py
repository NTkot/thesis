import os
import argparse
import subprocess
from common import RPI4_ROS_SRC, HOST_ROS_SRC, HOST_ROS_DOWNLOADED_SRC, RPI4_IP, ssh_command


def select_element_from_list(string_list):
    print("Select package to push to device:")
    for index, string in enumerate(string_list):
        print(f"{index + 1}. {string}")

    while True:
        choice = input("Enter the number corresponding to your choice: ")
        try:
            index = int(choice) - 1
            if 0 <= index < len(string_list):
                return string_list[index]
            else:
                print("Invalid choice. Please enter number within range")
        except ValueError:
            print("Invalid input. Please enter a positive integer")


def get_host_ros_pkgs():
    folder_names = [f for f in os.listdir(HOST_ROS_SRC) if os.path.isdir(os.path.join(HOST_ROS_SRC, f)) and (f != 'sync_rpi4') and (not f.startswith('.'))]

    return folder_names


def get_rpi4_ros_pkgs():
    cmd = "find " + RPI4_ROS_SRC + r' -maxdepth 1 -mindepth 1 -type d \( ! -name ".*" \) -printf "%f\n"'
    _, stdout, _ = ssh_command(cmd)
    stdout = stdout.read().decode('utf-8').strip()
    return stdout.split('\n')


def upload():
    print("Uploading...")

    folder_names = get_host_ros_pkgs()

    selected_folder = select_element_from_list(folder_names)

    host_src = HOST_ROS_SRC + "/" + selected_folder
    rpi4_src = "rpi4@" + RPI4_IP + ":" + RPI4_ROS_SRC

    stdout = subprocess.check_output(["rsync",
                                      "-rvt",
                                      "--exclude",
                                      "__pycache__",
                                      "--exclude",
                                      ".git*",
                                      host_src, 
                                      rpi4_src]).decode().strip()
    
    print(stdout)


def download():
    print("Downloading...")

    folder_names = get_rpi4_ros_pkgs()

    selected_folder = select_element_from_list(folder_names)

    rpi4_src = "rpi4@" + RPI4_IP + ":" + RPI4_ROS_SRC + "/" + selected_folder
    host_src = HOST_ROS_DOWNLOADED_SRC

    stdout = subprocess.check_output(["rsync",
                                      "-rvt",
                                      "--exclude",
                                      "__pycache__",
                                      "--exclude",
                                      ".git",
                                      rpi4_src, 
                                      host_src]).decode().strip()

    print(stdout)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                prog='sync_rpi4',
                description='Sync files to/from rpi4')
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-u', '--upload', 
                       default=False , 
                       action='store_true', 
                       help='upload files to rpi4 (used from local machine)')
    group.add_argument('-d', '--download', 
                       default=False, 
                       action='store_true', 
                       help='download files from rpi4 (used from rpi4)')
    
    args = parser.parse_args()

    if ((not args.upload) and (not args.download)) or args.upload:
        upload()
    else:
        download()
