import subprocess
from common import RPI4_EXTERNAL_ROSBAG_PATH, RPI4_DEFAULT_ROSBAG_PATH, HOST_ROSBAG_PATH, RPI4_IP


def download_bags():
    host_src = HOST_ROSBAG_PATH
    rpi4_external_rosbag_src = "rpi4@" + RPI4_IP + ":" + RPI4_EXTERNAL_ROSBAG_PATH
    rpi4_default_rosbag_src = "rpi4@" + RPI4_IP + ":" + RPI4_DEFAULT_ROSBAG_PATH

    print("======== Retrieving from default rosbag directory ========")
    subprocess.call(["rsync",
                     "-Prvth",
                     rpi4_default_rosbag_src,
                     host_src])
    
    print("======== Retrieving from external rosbag directory ========")
    subprocess.call(["rsync",
                     "-Prvth",
                     rpi4_external_rosbag_src,
                     host_src])



if __name__ == '__main__':
    download_bags()
