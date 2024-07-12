from __future__ import annotations
import os
import time
import numpy as np
from copy import deepcopy
from typing import Callable
from .file_wrapper import FileWrapper
from .labels import AnnotationList
from .segment import Segment, Vector3, Gps, Time
from .syncer import Syncer

    

class Data():
    def __init__(self, pickle_file: str, imu_topic = '/imu/calib', gps_topic = '/gps/gprmc', interp_method = 'linear'):
        wrapped_data = FileWrapper(pickle_file)
        self.data = Syncer.transform(wrapped_data, imu_topic, gps_topic, interp_method)

        self.metadata = {'file': os.path.basename(pickle_file),
                         'date': time.strftime('%A %d-%m-%Y %H:%M:%S', time.localtime(wrapped_data.minimum_time // 1e9)),
                         'time_points': self.__len__(),
                         'interp_method': interp_method,
                         'imu_topic': imu_topic,
                         'gps_topic': gps_topic}

        self.segments: list[Segment] = []

    def __getitem__(self, item: str) -> Vector3 | Gps | Time | np.ndarray:
        return self.data.__getitem__(item)
    
    def __len__(self):
        return len(self.data['time']['rel'])

    def segment(self, samples_per_segment: int, overlap_ratio: float = 0.0) -> list[Segment]:
        return self.data.segment(samples_per_segment, overlap_ratio)

    def transform(self, transform_fun: Callable[[np.ndarray], np.ndarray], vars_whitelist: list[list[str]] = None, in_place: bool = False) -> None | Data:
        # TODO: Add transform info to self.metadata
        ret = self.data.transform(transform_fun, vars_whitelist, in_place)
        if in_place:
            return None
        else:
            self_copy = deepcopy(self)
            self_copy.data = ret
            return self_copy


class LabeledData(Data):
    def __init__(self, pickle_file: str, annotation_file: str, imu_topic='/imu/calib', gps_topic='/gps/gprmc', interp_method='linear'):
        super().__init__(pickle_file, imu_topic, gps_topic, interp_method)

        # Update metadata with annotation filepath
        # For aesthetic reasons, add it second
        metadata_keys = list(self.metadata.keys())
        metadata_keys.insert(1, 'annotation_file')
        updated_metadata_dict = {}
        for key in metadata_keys:
            if key == 'annotation_file':
                updated_metadata_dict[key] = os.path.basename(annotation_file)
            else:
                updated_metadata_dict[key] = self.metadata[key]
        self.metadata = updated_metadata_dict

        self.annotations = AnnotationList()
        self.annotations.from_json(annotation_file)
        self.labels = self.annotations.to_time_samples(self.data['time']['rel'])
