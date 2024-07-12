import tqdm
import numpy as np
import requests
import warnings
from typing import Callable
from geopy import Nominatim
from unidecode import unidecode

from data_process.segment import Segment, Gps, Vector3
from data_process.signal_tools import signal_filter
from data_process.utils import get_by_path


class DocumentBuilder():
    """
        Class that contains functionality for constructing
        dictionaries containing features from data segments
        corresponding to road anomalies.
    """
    
    def __init__(self, doc_format: dict[str, tuple[list[list[str]], Callable]]):
        """
            Input dict `doc_format` should have the following format:
            {
                "feature1": tuple(list[list[str]], Callable),
                "feature2": tuple(list[list[str]], Callable),
                ...
                "featureN": tuple(list[list[str]], Callable),
            }

            First element in tuple is a list containing paths to 
            variables inside LabeledData, e.g. [['gps', 'lat'], 
            ['accel', 'z']].

            Second element in tuple is a function that is called
            to extract the document feature.
        """
        if not doc_format:
            raise ValueError("Document format dict can not be empty")
        self.doc_format = doc_format

    def transform(self, data_segment: Segment) -> dict:
        doc = {}
        for metric, metric_opts in self.doc_format.items():
            path_to_vars = metric_opts[0]
            metric_func  = metric_opts[1]
            doc[metric]  = metric_func(*[get_by_path(data_segment, path_to_var) for path_to_var in path_to_vars])
        return doc

    def transform_slices(self, data: Segment,
                               slices_range: list[tuple[int, int]],
                               preprocess_data: list[tuple[list[list[str]], Callable]] = []) -> list[dict]:
        if preprocess_data:
            processed_data = data
            for value in preprocess_data:
                processed_data.transform(value[1], value[0], in_place=True)
        else:
            processed_data = data

        out_list = []
        with tqdm.tqdm(total=len(slices_range)) as pbar:
            for slice in slices_range:
                data_segment = processed_data.slice(slice[0], slice[1]+1)
                out_list.append(self.transform(data_segment))
                pbar.update(1)
        return out_list


class DocumentFeatures():
    """
        Class containing functions for extracting features from
        data segments corresponding to anomalies, e.g.
        - Address info
        - Anomaly severity 
    """
    geolocator = Nominatim(user_agent="thesis")
    with open('/home/ntkot/theses/gapi.txt', 'r') as f:
        gapi = f.read()

    @classmethod
    def address_info_osm(cls, gps: Gps):
        location = cls.geolocator.reverse(f"{np.average(gps.lat)}, {np.average(gps.lon)}", language='en')

        out_dict = {}
        for key, value in location.raw['address'].items():
            if key == 'road':
                out_dict['street_name'] = unidecode(value)
            elif key == 'city':
                out_dict['city'] = unidecode(value)
            elif key == 'country':
                out_dict['country'] = unidecode(value)
            elif key == 'postcode':
                out_dict['zip_code'] = unidecode(value)
                
        return out_dict

    @classmethod
    def address_info_google(cls, gps: Gps) -> dict:
        response = requests.get(f'https://maps.googleapis.com/maps/api/geocode/json'
                                f'?latlng={np.average(gps.lat)},{np.average(gps.lon)}'
                                f'&result_type=street_address'
                                f'&key={cls.gapi}').json()

        if response['status'] != 'OK':
            warnings.warn(f"Location lat={np.average(gps.lat)},lon={np.average(gps.lon)} returned non-OK status")
            return {}
        else:
            return cls._parse_google_response(gps, response)

    @classmethod
    def _parse_google_response(cls, gps: Gps, response: dict) -> dict:
        best_result_idx = 0
        min_dist = np.sqrt((np.average(gps.lat) - response["results"][0]["geometry"]["location"]["lat"]) ** 2 + \
                           (np.average(gps.lon) - response["results"][0]["geometry"]["location"]["lng"]) ** 2)

        for i in range(1, len(response["results"])):
            result = response["results"][i]
            dist = np.sqrt((np.average(gps.lat) - result["geometry"]["location"]["lat"]) ** 2 + \
                           (np.average(gps.lon) - result["geometry"]["location"]["lng"]) ** 2)
            if dist < min_dist:
                best_result_idx = i
                min_dist = dist

        best_result = response["results"][best_result_idx]
        out_dict = {}
        for component in best_result['address_components']:
            if 'street_number' in component['types']:
                out_dict['street_number'] = component['long_name']
            elif 'route' in component['types']:
                out_dict['street_name'] = component['long_name']
            elif 'locality' in component['types']:
                out_dict['city'] = component['long_name']
            elif 'country' in component['types']:
                out_dict['country'] = component['long_name']
            elif 'postal_code' in component['types']:
                out_dict['zip_code'] = component['long_name']
        return out_dict
    
    @classmethod
    def accel_based_severity_info(self, accel: Vector3, gps: Gps):
        # def rms(a: np.ndarray):
        #     return np.sqrt(np.mean(a ** 2))
        # return ((max(accel.z) + rms(accel.z)) / 2) / (np.clip(np.average(gps.speed), 15.0, 70.0) / 3.6)
        return max(accel.z) / (np.clip(np.average(gps.speed), 15.0, 70.0) / 3.6)

    @classmethod
    def rel_time_range(self, rel_time: np.ndarray):
        return [min(rel_time), max(rel_time)]

    @classmethod
    def anomaly_length(self, rel_time: np.ndarray, gps_speed: np.ndarray):
        speed_ms = np.average(gps_speed) / 3.6
        return (max(rel_time) - min(rel_time)) * speed_ms
