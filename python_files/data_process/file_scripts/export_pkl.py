import os
import argparse
import compress_pickle
from data_process.file_wrapper import FileWrapper
from multiprocessing import Pool
from common import find_rosbags, prompt_user, BAG_DB_PATH


def save_to_pickle_task(param_dict):
    obj = FileWrapper(param_dict['rosbag_path'], load_images=param_dict['store_images'])

    pickle_filename = os.path.basename(os.path.normpath(param_dict['rosbag_path']))
    if param_dict['store_images']:
        pickle_filename += '_img'
    pickle_filename += '.pkl'
     
    obj.save_to_pickle(os.path.join(os.path.abspath(param_dict['rosbag_path']), pickle_filename), compression=param_dict['compression'])


def prompt_user_rosbags() -> list[int]:
    rosbag_dirs = find_rosbags()
    reduced_rosbag_dirs = [s.replace(BAG_DB_PATH, '') for s in rosbag_dirs]
    indices = prompt_user(reduced_rosbag_dirs, "Select rosbag directories to generate pickle file:")
    return [rosbag_dirs[i] for i in indices], [reduced_rosbag_dirs[i] for i in indices]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convert ROS2 bags and export it to a pickle file")

    parser.add_argument("--store-images", default=False, help="Add option to store images too", action="store_true")
    parser.add_argument("--compression",  default=None,  help="Compression method", choices=compress_pickle.get_known_compressions())

    args = parser.parse_args()

    selected_rosbag_dirs, selected_reduced_rosbag_dirs = prompt_user_rosbags()

    arguments_list = []
    for i in range(len(selected_rosbag_dirs)):
        arguments_list.append({'compression': args.compression,
                               'store_images': args.store_images,
                               'rosbag_path': selected_rosbag_dirs[i]})
    
    pool = Pool()
    for idx, _ in enumerate(pool.imap(save_to_pickle_task, arguments_list)):
        print(f"Exporting pickle from rosbag: '{selected_reduced_rosbag_dirs[i]}' completed ", flush=True)
