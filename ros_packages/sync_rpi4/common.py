import os
import paramiko

RPI4_IP = "road-quality.local"

RPI4_ROS_WS = "~/road_quality_ws"
HOST_ROS_WS = os.environ['HOME'] + "/theses/ros"

RPI4_ROS_SRC = RPI4_ROS_WS + "/src"
HOST_ROS_SRC = HOST_ROS_WS + "/src"
HOST_ROS_DOWNLOADED_SRC = HOST_ROS_WS + "/downloaded_src"

RPI4_EXTERNAL_ROSBAG_PATH = "/mnt/usb/bag_db"
RPI4_DEFAULT_ROSBAG_PATH = "~/bag_db"
HOST_ROSBAG_PATH = HOST_ROS_WS


def get_ssh_object():
    ssh = paramiko.SSHClient()
    key = paramiko.RSAKey.from_private_key_file(os.path.expanduser("~/.ssh/id_rsa"))
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname=RPI4_IP, username='rpi4', pkey=key)
    return ssh


def ssh_command(cmd):
    ssh_obj = get_ssh_object()
    return ssh_obj.exec_command(cmd)


def get_directory_names(path):
    names = os.listdir(path)
    ret = []
    for name in names:
        if os.path.isdir(os.path.join(path,name)):
            ret.append(name)
    return ret
