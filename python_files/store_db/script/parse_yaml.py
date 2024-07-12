import os
import yaml
import inspect
import importlib
from pprint import pprint

from store_db.mongo_wrapper import MongoWrapper
from store_db.doc_builder import DocumentBuilder
from store_db.doc_aggregator import DocumentAggregator

default_yaml = os.path.join(os.path.dirname(__file__), 'options.yaml')


def parse_doc_builder_yaml(file_overwrite = '') -> DocumentBuilder:
    file = file_overwrite if file_overwrite else default_yaml
    with open(file) as stream:
        try:
            config = yaml.safe_load(stream)['doc_builder_format']
        except yaml.YAMLError as exc:
            print(exc)

    # pprint(config, sort_dicts=False)
    doc_format = {}
    for feature in config:
        feature_name = feature["name"]
        module = importlib.import_module(feature["module"])
        function = module
        for path in feature["function"]:
            function = getattr(function, path)
        doc_format[feature_name] = (feature["var_path"], function)

    # pprint(doc_format, sort_dicts=False)
    return DocumentBuilder(doc_format)


def parse_doc_aggregator_yaml(file_overwrite = '') -> DocumentAggregator:
    file = file_overwrite if file_overwrite else default_yaml
    with open(file) as stream:
        try:
            config = yaml.safe_load(stream)['doc_aggregator_format']
        except yaml.YAMLError as exc:
            print(exc)

    # pprint(config, sort_dicts=False)
    doc_format = {}
    for feature in config:
        feature_name = feature["name"]
        module = importlib.import_module(feature["module"])
        function = module
        for path in feature["function"]:
            function = getattr(function, path)
        doc_format[feature_name] = function

    # pprint(doc_format, sort_dicts=False)
    return DocumentAggregator(doc_format)


def parse_mongo_options_yaml(file_overwrite = '') -> dict:
    file = file_overwrite if file_overwrite else default_yaml
    with open(file) as stream:
        try:
            config = yaml.safe_load(stream)['mongo_options']
        except yaml.YAMLError as exc:
            print(exc)

    signature = inspect.signature(MongoWrapper.__init__)
    default_args = {
        k: v.default
        for k, v in signature.parameters.items()
        if v.default is not inspect.Parameter.empty
    }

    if config is not None:
        for key in default_args.keys():
            if key in config.keys() and (config[key] is not None and config[key]):
                default_args[key] = config[key]

    # pprint(default_args, sort_dicts=False)
    return default_args


def parse_preprocess_options_yaml(file_overwrite = ''):
    file = file_overwrite if file_overwrite else default_yaml
    with open(file) as stream:
        try:
            config = yaml.safe_load(stream)['preprocess']
        except yaml.YAMLError as exc:
            print(exc)
    # pprint(config, sort_dicts=False)

    return config


def parse_detector_options_yaml(file_overwrite = ''):
    file = file_overwrite if file_overwrite else default_yaml
    with open(file) as stream:
        try:
            config = yaml.safe_load(stream)['detector']
        except yaml.YAMLError as exc:
            print(exc)
    # pprint(config, sort_dicts=False)

    module = importlib.import_module(config["module"])
    cls = getattr(module, config["class"])

    if "ignore_ranges" in config.keys():
        ignore_ranges = config["ignore_ranges"]
    else:
        ignore_ranges = []

    return cls(**config["args"]), config["var_path"], ignore_ranges




if __name__ == '__main__':
    # print(parse_doc_builder_yaml())

    # print(parse_doc_aggregator_yaml())

    # pprint(parse_mongo_options_yaml(), sort_dicts=False)

    # pprint(parse_preprocess_options_yaml(), sort_dicts=False)

    detector, var_paths, ignore_ranges = parse_detector_options_yaml()
    # print(detector.start_value_threshold)
    # print(detector.stop_value_threshold)
    # print(detector.start_counter_threshold)
    # print(detector.stop_counter_threshold)
    # print(var_paths)
    print(ignore_ranges)
    # print(type(detector))

    print()
