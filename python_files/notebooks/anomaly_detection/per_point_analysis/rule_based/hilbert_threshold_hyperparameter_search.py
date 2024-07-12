import os
import json
import numpy as np
import multiprocessing
from datetime import datetime
from tqdm import tqdm
from itertools import product

from data_process.data import LabeledData
from data_process.labels import AnomalyLabel
from data_process.signal_tools import signal_filter, signal_transform
from data_process.utils import ClassifierMetrics
from data_process.detect.rule_based.envelope_threshold import EnvelopeThreshold


class HyperparameterSearch:
    @staticmethod
    def hyperparameter_combinations(hyperparameters: dict):
        for params in product(hyperparameters['filter']['accel']['wiener'],
                              hyperparameters['filter']['accel']['hp_order'],
                              hyperparameters['filter']['accel']['hp_freq'],
                              hyperparameters['filter']['gyro']['wiener'],
                              hyperparameters['filter']['gyro']['hp_order'],
                              hyperparameters['filter']['gyro']['hp_freq'],
                              hyperparameters['detection']['start_value_threshold'],
                              hyperparameters['detection']['stop_value_threshold'],
                              hyperparameters['detection']['start_counter_threshold'],
                              hyperparameters['detection']['stop_counter_threshold']):
            yield \
            {
                'filter':
                {
                    'accel':
                    {
                        'wiener': params[0],
                        'hp_order': params[1],
                        'hp_freq': params[2]
                    },
                    'gyro':
                    {
                        'wiener': params[3],
                        'hp_order': params[4],
                        'hp_freq': params[5]
                    }
                },
                'detection':
                {
                    'start_value_threshold'  : params[6],
                    'stop_value_threshold'   : params[7],
                    'start_counter_threshold': params[8],
                    'stop_counter_threshold' : params[9]
                }
            }

    @staticmethod
    def filter_data(original_data: LabeledData, sample_rate: float, params: dict):
        accel_wiener = lambda x: signal_filter.wiener(x, params['filter']['accel']['wiener'])
        gyro_wiener  = lambda x: signal_filter.wiener(x, params['filter']['gyro']['wiener'])

        accel_hp_filter = lambda x: signal_filter.butterworth(signal=x, 
                                                              sample_rate=sample_rate, 
                                                              order=params['filter']['accel']['hp_order'],
                                                              critical_freq=params['filter']['accel']['hp_freq'],
                                                              btype='highpass')
        gyro_hp_filter  = lambda x: signal_filter.butterworth(signal=x, 
                                                              sample_rate=sample_rate, 
                                                              order=params['filter']['gyro']['hp_order'], 
                                                              critical_freq=params['filter']['gyro']['hp_freq'],
                                                              btype='highpass')

        envelope_fun = lambda x: np.abs(signal_transform.hilbert(x))

        out_data = original_data.transform(accel_wiener, [['accel', 'x'], ['accel', 'y'], ['accel', 'z']])
        out_data.transform(gyro_wiener, [['gyro', 'x'], ['gyro', 'y'], ['gyro', 'z']], in_place=True)

        out_data.transform(accel_hp_filter, [['accel', 'x'], ['accel', 'y'], ['accel', 'z']], in_place=True)
        out_data.transform(gyro_hp_filter,  [['gyro',  'x'], ['gyro',  'y'], ['gyro',  'z']], in_place=True)

        out_data.transform(envelope_fun, [['accel', 'x'], ['accel', 'y'], ['accel', 'z'], 
                                          ['gyro',  'x'], ['gyro',  'y'], ['gyro',  'z']], in_place=True)

        return out_data
    
    @staticmethod
    def metrics(envelope_data: LabeledData, detector: EnvelopeThreshold):
        y_true = [1 
                  if point_labels and (not AnomalyLabel.NO_ANOMALY in [label.anomaly for label in point_labels]) 
                  else 0 
                  for point_labels in envelope_data.labels]
        y_pred = detector.to_time_samples(envelope_data['time']['rel'])

        cm = ClassifierMetrics.confusion_matrix(y_true, y_pred, ignore_ranges=ignore_ranges_classification)
        overall_metrics = ClassifierMetrics.overall_metrics_from_cm(cm)
        class_metrics = ClassifierMetrics.class_metrics_from_cm(cm)

        return cm, overall_metrics, class_metrics
    
    @staticmethod
    def hyperparameter_search_task(args_dict):
        original_data = args_dict['original_data']
        params = args_dict['params']

        envelope_data = HyperparameterSearch.filter_data(original_data, sample_rate, params)

        detector = EnvelopeThreshold(start_value_threshold=params['detection']['start_value_threshold'],
                                        stop_value_threshold=params['detection']['stop_value_threshold'],
                                        start_counter_threshold=params['detection']['start_counter_threshold'],
                                        stop_counter_threshold=params['detection']['stop_counter_threshold'])
        detector.predict(envelope_data['accel']['z'])

        cm, overall_metrics, class_metrics = HyperparameterSearch.metrics(envelope_data, detector)

        stats_dict = \
        {
            "params": params,
            "metrics":
            {
                "cm": [int(cm[0,0]), int(cm[0,1]), int(cm[1,0]), int(cm[1,1])],
                "overall":
                {
                    "accuracy":   overall_metrics["accuracy"],
                    "precision":  overall_metrics["precision"],
                    "recall":     overall_metrics["recall"],
                    "f1_measure": overall_metrics["f1_measure"]
                },
                "anomalies":
                {
                    "accuracy":   class_metrics["accuracy"][1],
                    "precision":  class_metrics["precision"][1],
                    "recall":     class_metrics["recall"][1],
                    "f1_measure": class_metrics["f1_measure"][1]
                },
                "non_anomalies":
                {
                    "accuracy":   class_metrics["accuracy"][0],
                    "precision":  class_metrics["precision"][0],
                    "recall":     class_metrics["recall"][0],
                    "f1_measure": class_metrics["f1_measure"][0]
                }
            }
        }
        return stats_dict

    


if __name__ == '__main__':
    pickle_file = '/home/ntkot/theses/ros/bag_db/data_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10.pkl'
    annotation_file = '/home/ntkot/theses/ros/bag_db/data_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10_labels_v2.json'
    ignore_ranges_classification = [(185906, 198500)]

    # HYPERPARAMETERS
    hyperparameters = \
    {
        'filter':
        {
            'accel':
            {
                'wiener': [21, 41],
                'hp_order': [2, 3, 4],
                'hp_freq': [1, 2, 3]
            },
            'gyro':
            {
                'wiener': [21, 41],
                'hp_order': [2, 3, 4],
                'hp_freq': [1, 2, 3]
            }
        },
        'detection':
        {
            'start_value_threshold'  : [0.11, 0.125, 0.15],
            'stop_value_threshold'   : [0.11, 0.125, 0.15],
            'start_counter_threshold': [5, 8, 11],
            'stop_counter_threshold' : [5, 8, 11]
        }
    }

    original_data = LabeledData(pickle_file, annotation_file)
    sample_rate = 1 / np.average(original_data['time']['rel'][1:] - original_data['time']['rel'][0:-1])

    arguments_list = []
    for params in HyperparameterSearch.hyperparameter_combinations(hyperparameters):
        arguments_list.append({'original_data': original_data, 'params': params})

    total_combinations = len(arguments_list)
    print(f'Total combinations: {total_combinations}')

    results = [{} for i in range(total_combinations)]
    date = datetime.now()
    pool = multiprocessing.Pool(processes=6)
    processes_done = 0
    last_update = 0
    with tqdm(total=total_combinations) as pbar:
        for i, result in enumerate(pool.imap(HyperparameterSearch.hyperparameter_search_task, arguments_list)):
            results[i] = result

            processes_done += 1
            if (processes_done - last_update) > (total_combinations // 10000):
                pbar.update(processes_done - last_update)
                last_update = processes_done
    
    filepath = os.path.join(os.path.dirname(__file__), f'hilbert_threshold_results_{date.strftime("%d_%m_%Y__%H_%M_%S")}.json')
    with open(filepath, 'w+') as file:
        json.dump(results, file, indent=4)
