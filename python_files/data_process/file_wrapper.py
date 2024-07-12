import os
import yaml
import tqdm
import nml_bag
import warnings
import collections
import numpy as np
from .utils import get_by_path, set_by_path, CompressPickleWrapper


class FileWrapper():
    def __init__(self, datapath, **kwargs):
        if os.path.exists(datapath):
            if os.path.isdir(datapath):
                print("Loading from rosbag...")
                self.load_from_bag(datapath, load_images=kwargs.get('load_images', False))
            elif os.path.isfile(datapath) and ('.pkl' in os.path.basename(datapath)):
                print("Loading from pickle file...")
                self.load_from_pickle(datapath)
            else:
                raise ValueError(f"Path '{datapath}' does not point to a rosbag folder nor a pickle exported file")
        else:
            raise ValueError(f"Path '{datapath}' does not exist")
        self._check_time_entries()

    def load_from_bag(self, bagpath, load_images: bool):
        self.data_lengths = {}
        
        # Find data lengths
        with open(os.path.join(bagpath, 'metadata.yaml')) as f:
            metadata = yaml.load(f, Loader=yaml.FullLoader)
            for entry in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
                self.data_lengths[entry['topic_metadata']['name']] = entry['message_count']
            total_records = metadata['rosbag2_bagfile_information']['message_count']
                
        # Actual data parsing
        self.data = {}
        self.data_indices = {}
        reader = nml_bag.Reader(bagpath)
        i = 0
        last_update = 0
        with tqdm.tqdm(total=total_records) as pbar:
            while True:
                if i % (total_records // 100) == 0:
                    # print(f"{i}/{total_records}")
                    pbar.update(i - last_update)
                    last_update = i
                i += 1
                try:
                    record = next(reader)
                    if (record['topic'] == '/image_raw/compressed' and load_images) or \
                       (not 'image_raw' in record['topic']):
                            self._parse_bag_record(record, [record['topic']])
                                
                except StopIteration:
                    break
                    
                except ModuleNotFoundError as e:
                    if 'theora' in e.msg:
                        warnings.warn('Ignoring theora missing packet error')
                    else:
                        raise(e)
                    
    def _check_time_entries(self) -> bool:
        ok = True
        tree_time_elements = [elem for elem in self.tree() if 'time_ns' in elem]

        time_start_list = []
        time_end_list = []
        for time_element in tree_time_elements:
            time_start_list.append(get_by_path(self.data, time_element)[0])
            time_end_list.append(get_by_path(self.data, time_element)[-1])
        self.minimum_time: int = min(time_start_list)
        self.maximum_time: int = max(time_end_list)

        for time_element in tree_time_elements:
            time_vector = get_by_path(self.data, time_element)
            time_vector = time_vector - self.minimum_time
            set_by_path(self.data, [*time_element[:-1], 'time'], time_vector.astype(float) / 1e9)

            time_diff = time_vector[1:] - time_vector[0:-1]

            if np.any(time_diff < 0):
                warnings.warn(f"Detected negative time jump in {time_element}")
                ok = False

        return ok

    def _parse_bag_record(self, record, key_list):
        if not (key_list[-1] in get_by_path(self.data, key_list[:-1]).keys()):
            set_by_path(self.data, key_list, {})
            set_by_path(self.data_indices, key_list, {})

        for key in record.keys():
            record_type = type(record[key])

            if key == 'topic' or key == 'header' or key == 'type' or key == 'stamp':
                continue
            elif (record_type == float) or (record_type == int) or \
                 (record_type == bool) or (record_type == str) or (record_type == list):
                if not (key in get_by_path(self.data, key_list).keys()):
                    set_by_path(self.data, [*key_list, key], np.empty(shape=self.data_lengths[key_list[0]], dtype=record_type))
                    set_by_path(self.data_indices, [*key_list, key], 0)

                i = get_by_path(self.data_indices, [*key_list, key])
                if '/image_raw/compressed' in key_list and key == 'data':
                    set_by_path(self.data, [*key_list, key, i], bytes(record[key]))
                else:
                    set_by_path(self.data, [*key_list, key, i], record[key])
                set_by_path(self.data_indices, [*key_list, key], i+1)
            elif (record_type == collections.OrderedDict):
                self._parse_bag_record(record[key], [*key_list, key])
            else:
                print(f'Record type: {record_type}, path: {[*key_list, key]}')

    def load_from_pickle(self, filepath):
        self.data = CompressPickleWrapper.load(filepath)

    def save_to_pickle(self, filepath, compression:str=None):
        CompressPickleWrapper.save(self.data, filepath, compression)
        
    def tree(self):
        self._tree = []
        self._gen_tree_level([])
        return self._tree

    def _gen_tree_level(self, path_to_master_key):
        iter = get_by_path(self.data, path_to_master_key).items()
        for key, value in iter:
            if isinstance(value, dict):
                self._gen_tree_level([*path_to_master_key, key])
            elif isinstance(value, np.ndarray):
                self._tree.append([*path_to_master_key, key])
            else:
                raise TypeError("Found invalid type inside data wrapper when generating tree")
            
    def is_static_calibrated(self) -> bool:
        return any('/imu/static_calib' in key for key in self.tree())
    
    def is_calibrated(self) -> bool:
        return any('/imu/calib' in key for key in self.tree())
