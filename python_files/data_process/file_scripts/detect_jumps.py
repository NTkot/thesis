import argparse
import matplotlib.pyplot as plt
from data_process.file_wrapper import FileWrapper
from data_process.utils import get_by_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Detect time jumps in data using a time threshold and a topic's timestamps")

    parser.add_argument("path_to_data", default="", help="Path to data that can be parsed using data_manip.FileWrapper", type=str)
    parser.add_argument("--time-threshold", default=0.1, type=float, help="Time differences between messages above this threshold will be considered time jumps")
    parser.add_argument("--topic", default="/imu/raw", type=str, help="Topic to use")

    args = parser.parse_args()

    data = FileWrapper(args.path_to_data)
    time = get_by_path(data.data, [args.topic, 'time'])

    time_diff = time[1:] - time[0:(len(time)-1)]
    idx = (time_diff > args.time_threshold).nonzero()[0]

    last_idx = -1
    jumps_idx_start = []
    for i in idx:
        if i != (last_idx + 1):
            jumps_idx_start.append(i-1)
        last_idx = i
    
    plt.plot(time_diff)
    for i in jumps_idx_start:
        plt.axvline(x=i, color='r')
        print(f"Detected time jump in {time[i]:.4f}s (index: {i})")
    plt.show()
