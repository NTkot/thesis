import argparse
import sys
import numpy as np
from typing import Callable

from data_process.data import Data, LabeledData
from data_process.signal_tools import signal_filter, signal_transform
from data_process.utils import get_by_path
from store_db.mongo_wrapper import MongoWrapper
from store_db.script.parse_yaml import parse_doc_builder_yaml, parse_doc_aggregator_yaml, \
                                       parse_mongo_options_yaml, parse_detector_options_yaml, \
                                       parse_preprocess_options_yaml


def create_mongo_wrapper(options_arg) -> MongoWrapper:
    doc_builder = parse_doc_builder_yaml(options_arg)
    doc_aggregator = parse_doc_aggregator_yaml(options_arg)
    rest_of_args = parse_mongo_options_yaml(options_arg)

    return MongoWrapper(doc_builder=doc_builder,
                        doc_aggregator=doc_aggregator,
                        **rest_of_args)


def preprocess_functions(options_arg: str, sample_rate: float) -> list[tuple[list[list[str], Callable]]]:
    config = parse_preprocess_options_yaml(options_arg)

    accel_wiener_window  = config['accel']['wiener_window']
    accel_highpass_order = config['accel']['highpass_order']
    accel_highpass_critical_frequency = config['accel']['highpass_freq']

    gyro_wiener_window  = config['gyro']['wiener_window']
    gyro_highpass_order = config['gyro']['highpass_order']
    gyro_highpass_critical_frequency = config['gyro']['highpass_freq']

    accel_filter = lambda x: signal_filter.butterworth(signal=signal_filter.wiener(x, accel_wiener_window),
                                                       sample_rate=sample_rate,
                                                       order=accel_highpass_order,
                                                       critical_freq=accel_highpass_critical_frequency,
                                                       btype='highpass')
    gyro_filter  = lambda x: signal_filter.butterworth(signal=signal_filter.wiener(x, gyro_wiener_window),
                                                       sample_rate=sample_rate,
                                                       order=gyro_highpass_order,
                                                       critical_freq=gyro_highpass_critical_frequency,
                                                       btype='highpass')
    
    envelope_fun = lambda x: np.abs(signal_transform.hilbert(x))

    return [([['accel', 'x'], ['accel', 'y'], ['accel', 'z']], accel_filter), 
            ([['gyro',  'x'], ['gyro',  'y'], ['gyro',  'z']], gyro_filter),
            ([['accel', 'x'], ['accel', 'y'], ['accel', 'z'],
              ['gyro',  'x'], ['gyro',  'y'], ['gyro',  'z']], envelope_fun)]


def store(args, sub_args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, type=str)
    parser.add_argument('--annotation', type=str)
    parser.add_argument('--first', type=int)
    parser.add_argument('--preprocess', action='store_true')
    sub_args = parser.parse_args(sub_args)

    if sub_args.annotation:
        print('Annotation file specified, using annotations for map...')
        data = LabeledData(sub_args.data, sub_args.annotation)

        annotation_slices = [(ret[1], ret[2]) for ret in data.annotations.to_time_range_indices(data['time']['rel'])]
        if sub_args.first:
            annotation_slices = annotation_slices[:sub_args.first]

        if sub_args.preprocess:
            sample_rate = 1 / np.average(data['time']['rel'][1:] - data['time']['rel'][0:-1])
            preprocess_list = preprocess_functions(args.options, sample_rate)
        else:
            preprocess_list = []

        mongo_wrapper = create_mongo_wrapper(args.options)
        mongo_wrapper.add_slices(data.data, annotation_slices, preprocess_list)
    else:
        print('Annotation file not specified, running detection on data file...')
        data = Data(sub_args.data)
        detector, var_paths, ignore_ranges = parse_detector_options_yaml(args.options)

        if sub_args.preprocess:
            sample_rate = 1 / np.average(data['time']['rel'][1:] - data['time']['rel'][0:-1])
            preprocess_list = preprocess_functions(args.options, sample_rate)
            for preprocess_step in preprocess_list:
                path = preprocess_step[0]
                function = preprocess_step[1]
                data.transform(function, path, in_place=True)

        detector.predict(*[get_by_path(data.data, path) for path in var_paths], ignore_ranges)
        detected_slices = [(start, stop) for start, stop in zip(detector.anomalies_start_idx, detector.anomalies_stop_idx)]
        if sub_args.first:
            detected_slices = detected_slices[:sub_args.first]

        mongo_wrapper = create_mongo_wrapper(args.options)
        mongo_wrapper.add_slices(data.data, detected_slices)


def plot(args, sub_args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw', action='store_true', default=False)
    sub_args = parser.parse_args(sub_args)

    mongo_wrapper = create_mongo_wrapper(args.options)
    mongo_wrapper.plot(raw_db=sub_args.raw)


def cluster(args):
    mongo_wrapper = create_mongo_wrapper(args.options)
    mongo_wrapper._cluster_raw_collection()


def clear(args):
    mongo_wrapper = create_mongo_wrapper(args.options)
    mongo_wrapper._clear()


def stats(args):
    mongo_wrapper = create_mongo_wrapper(args.options)
    db = mongo_wrapper.store_options["db"]
    raw_coll = mongo_wrapper.store_options["raw_collection"]
    main_coll = mongo_wrapper.store_options["main_collection"]
    print(f'Raw collection:\n'
          f'  Total docs: {mongo_wrapper.mongoc[db][raw_coll].count_documents({})}\n'
          f'Main collection:\n'
          f'  Total docs: {mongo_wrapper.mongoc[db][main_coll].count_documents({})}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("function",
                        nargs='?',
                        choices=['store', 'plot', 'cluster', 'clear', 'stats'])
    parser.add_argument('--help', '-h', action='store_true')
    parser.add_argument('--options', type=str, default='', help='(Optional) Path to a YAML file, like options.yaml')
    args, sub_args = parser.parse_known_args()

    # print(f'Function arg: {args.function}, help arg: {args.help}')
    if args.help:
        # If no subcommand was specified, give general help
        if args.function is None: 
            print(parser.format_help())
            sys.exit(0)
        # Otherwise pass the help option on to the subcommand
        sub_args.append('--help')

    if args.function == "store":
        store(args, sub_args)
    elif args.function == "plot":
        plot(args, sub_args)
    elif args.function == "cluster":
        cluster(args)
    elif args.function == "clear":
        clear(args)
    elif args.function == "stats":
        stats(args)
