import os
import numpy as np
import json
import pickle

from data_process.file_wrapper import FileWrapper
from data_process.utils import get_by_path
from common import find_pickles, prompt_user, prompt_user_yes_no, BAG_DB_PATH


def generate_dataset_file(pickle_file, annotation_file = ''):
    """
        Returns a dictionary with the following format:
        {
            'imu':
            {
                'accel':
                {
                    'x': np.ndarray[float],     # in m/s^2
                    'y': np.ndarray[float],     # in m/s^2
                    'z': np.ndarray[float]      # in m/s^2
                },
                'gyro':
                {
                    'x': np.ndarray[float],     # in rad/s
                    'y': np.ndarray[float],     # in rad/s
                    'z': np.ndarray[float]      # in rad/s
                },
                'time':
                {
                    'rel': np.ndarray[float],       # in sec
                    'abs': np.ndarray[numpy.int64]  # in nanosec
                }
            },
            'gps':
            {
                'lon': np.ndarray[float],       # in degrees
                'lat': np.ndarray[float],       # in degrees
                'speed': np.ndarray[float],     # in m/s
                'track': np.ndarray[float],     # in degrees
                'time':
                {
                    'rel': np.ndarray[float],       # in sec
                    'abs': np.ndarray[numpy.int64]  # in nanosec
                }
            },
            'camera':
            {
                'data': np.ndarray[bytes],      # JPEG bytes
                'time':
                {
                    'rel': np.ndarray[float],       # in sec
                    'abs': np.ndarray[numpy.int64]  # in nanosec
                }
            }
            'labels':   # OPTIONAL
            [
                {
                    ...
                },
                {
                    'rel_t_start': float,   # in sec
                    'rel_t_end': float,     # in sec
                    'anomaly': str,         # one of 'no_anomaly', 'manhole', 'depression', 'bump', 'crack'
                    'transversity': str,    # one of 'no_transverse', 'transverse'
                    'severity': str         # one of 'small_severity', 'medium_severity', 'high_severity'
                },
                {
                    ...
                },
            ]
        }
    """
    file_wrapper = FileWrapper(pickle_file, load_images=True)
    # print(*file_wrapper.tree(), sep='\n')

    # out_name = os.path.basename(pickle_file).split('.pkl')[0] + '_dataset.pkl'
    out_name = 'dataset.pkl'
    out_dict = {'imu': {}, 'gps': {}}

    if not any(['/imu/calib' in topic for topic in file_wrapper.tree()]):
        raise ValueError(f"No calibrated IMU topic in file {os.path.basename(pickle_file)}")
    else:
        out_dict['imu']['accel'] = {'x': get_by_path(file_wrapper.data, ['/imu/calib', 'linear_acceleration', 'x']),
                                    'y': get_by_path(file_wrapper.data, ['/imu/calib', 'linear_acceleration', 'y']),
                                    'z': get_by_path(file_wrapper.data, ['/imu/calib', 'linear_acceleration', 'z'])}
        out_dict['imu']['gyro']  = {'x': get_by_path(file_wrapper.data, ['/imu/calib', 'angular_velocity',    'x']),
                                    'y': get_by_path(file_wrapper.data, ['/imu/calib', 'angular_velocity',    'y']),
                                    'z': get_by_path(file_wrapper.data, ['/imu/calib', 'angular_velocity',    'z'])}
        out_dict['imu']['time']  = {'rel': get_by_path(file_wrapper.data, ['/imu/calib', 'time']),
                                    'abs': get_by_path(file_wrapper.data, ['/imu/calib', 'time_ns'])}
        
    if not any(['/gps/gprmc' in topic for topic in file_wrapper.tree()]):
        raise ValueError(f"No GPS - GPRMC topic in file {os.path.basename(pickle_file)}")
    else:
        out_dict['gps']['lon']   = get_by_path(file_wrapper.data, ['/gps/gprmc', 'longitude_deg'])
        out_dict['gps']['lat']   = get_by_path(file_wrapper.data, ['/gps/gprmc', 'latitude_deg'])
        out_dict['gps']['speed'] = get_by_path(file_wrapper.data, ['/gps/gprmc', 'ground_speed_kmh']) / 3.6
        out_dict['gps']['track'] = get_by_path(file_wrapper.data, ['/gps/gprmc', 'track_deg'])
        out_dict['gps']['time']  = {'rel': get_by_path(file_wrapper.data, ['/gps/gprmc', 'time']),
                                    'abs': get_by_path(file_wrapper.data, ['/gps/gprmc', 'time_ns'])}
        
    if not any(['/image_raw/compressed' in topic for topic in file_wrapper.tree()]):
        print(f"WARNING: No CAMERA topic in file {os.path.basename(pickle_file)}")
    else:
        out_dict['camera'] = {}
        out_dict['camera']['data'] = get_by_path(file_wrapper.data, ['/image_raw/compressed', 'data'])
        out_dict['camera']['time'] = {'rel': get_by_path(file_wrapper.data, ['/image_raw/compressed', 'time']),
                                      'abs': get_by_path(file_wrapper.data, ['/image_raw/compressed', 'time_ns'])}
        
    if annotation_file:
        out_dict['labels'] = []
        with open(annotation_file, 'rb') as json_file:
            annotations = json.load(json_file)['annotations']
            for ann in annotations:
                label_dict = {
                    'rel_t_start': ann['rel_time_start'],
                    'rel_t_end': ann['rel_time_end'],
                    'anomaly': ann['label']['anomaly']['type'].lower(),
                    'transversity': ann['label']['transversity']['type'].lower(),
                    'severity': ann['label']['severity']['type'].lower() 
                }
                out_dict['labels'].append(label_dict)

    with open(os.path.join(os.path.dirname(pickle_file), out_name), 'wb') as dumped_file:
        pickle.dump(out_dict, dumped_file)


def prompt_user_pickles() -> list[str]:
    paths2search = [BAG_DB_PATH, os.getcwd()]

    for path in paths2search:
        pickle_dirs = find_pickles(path)
        if pickle_dirs:
            reduced_rosbag_dirs = [s.replace(path, '') for s in pickle_dirs]
            indices = prompt_user(reduced_rosbag_dirs, "Select pickle file to annotate:")
            return [pickle_dirs[i] for i in indices]
    
    raise ValueError("Could not auto-detect pickle files on your system. Specify path using --pickle-file option")


def prompt_user_annotation_file(selected_pickle) -> str:
    ans = prompt_user_yes_no('Do you want to include annotations?')
    if ans:
        path2search = os.path.dirname(selected_pickle)

        annotation_files = []
        for file in os.listdir(path2search):
            if os.path.isfile(os.path.join(path2search, file)) and 'labels' in file and file.endswith('.json'):
                annotation_files.append(file)

        if annotation_files:
            selected_annotation_idx = prompt_user(annotation_files, "Select annotation file:")
            if len(selected_annotation_idx) != 1:
                raise ValueError("You need to select exactly one annotation file")
            return os.path.join(path2search, annotation_files[selected_annotation_idx[0]])
        else:
            print('No annotation files found')
            return ''
    else:
        print('Not including annotations')
        return ''

    


if __name__ == '__main__':
    selected_pickle = prompt_user_pickles()

    if len(selected_pickle) != 1:
        raise ValueError("You need to select exactly one pickle")
    selected_pickle = selected_pickle[0]

    selected_annotation = prompt_user_annotation_file(selected_pickle)

    generate_dataset_file(selected_pickle, selected_annotation)
