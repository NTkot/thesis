import os
import json
import argparse
from data_process.labels import AnnotationList

FILENAME_JSON_KEY = 'file'
DATETIME_JSON_KEY = 'date'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Combine multiple annotation files into a single one",
                                     usage='%(prog)s [-h] file file [file ...]')
    parser.add_argument('file1', nargs=1,   metavar='file')
    parser.add_argument('file2', nargs='+', metavar='file', help=argparse.SUPPRESS)
    parser.add_argument('output_file')
    args = parser.parse_args()

    files = args.file1 + args.file2

    # Check for file existence
    for file in files:
        if not os.path.isfile(file):
            raise FileNotFoundError(f"Could not find: {file}")

    # Load annotations from each file
    list_of_ann_lists = []
    list_of_annotated_files = []
    list_of_annotated_files_dates = []
    for file in files:
        with open(file, 'r', encoding='utf-8') as f:
            in_dict = json.load(f)
            list_of_annotated_files.append(in_dict[FILENAME_JSON_KEY])
            list_of_annotated_files_dates.append(in_dict[DATETIME_JSON_KEY])
        ann_list = AnnotationList()
        ann_list.from_json(file)
        list_of_ann_lists.append(ann_list)

    # Check if all files have same annotated file/date
    same_file = all(i == list_of_annotated_files[0]       for i in list_of_annotated_files)
    same_date = all(i == list_of_annotated_files_dates[0] for i in list_of_annotated_files_dates)
    if not same_file:
        raise ValueError('JSON input files have different data file as reference')
    if not same_date:
        raise ValueError('JSON input files have different data date as reference')

    # Combine into one AnnotationList
    out_list = AnnotationList()
    for ann_list in list_of_ann_lists:
        out_list = AnnotationList.combine(out_list, ann_list)
    
    # Export to output file
    # print(json.dumps(out_list.to_dict(), indent=4))
    out_list.to_json(args.output_file)
