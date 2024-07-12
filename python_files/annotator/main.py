import os
import sys
import argparse
from annotator import Annotator
from PyQt5 import QtWidgets
from data_process.file_scripts.common import BAG_DB_PATH, find_pickles, prompt_user


def prompt_user_pickles() -> list[str]:
    paths2search = [BAG_DB_PATH]
    print(paths2search)

    for path in paths2search:
        pickle_dirs = find_pickles(path)
        if pickle_dirs:
            reduced_rosbag_dirs = [s.replace(path, '') for s in pickle_dirs]
            indices = prompt_user(reduced_rosbag_dirs, "Select pickle file to annotate:")
            return [pickle_dirs[i] for i in indices]
    
    raise ValueError("Could not auto-detect pickle files on your system. Specify path using --pickle-file option")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pickle-file", required=False, help="Pickle file to load data from", type=str)
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    if args.pickle_file is None:
        paths = prompt_user_pickles()
        if len(paths) > 1:
            raise ValueError("You need to select exactly one pickle file")
        pickle_file = paths[0]
    else:
        pickle_file = args.pickle_file

    annotator_obj = Annotator(pickle_file)
    app.exec_()
    sys.exit()


if __name__ == '__main__':
    main()
