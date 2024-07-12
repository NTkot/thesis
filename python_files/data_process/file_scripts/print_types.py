import sys
from data_process.file_wrapper import FileWrapper
from data_process.utils import get_by_path

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise ValueError('Provide exactly one argument (path to pickle file)')
    path = sys.argv[1]
    
    obj = FileWrapper(path)
    for key in obj.tree():
        element = get_by_path(obj.data, key)
        arr_type = type(element)
        sub_type = type(element[0])
        size = len(element)
        print(f'{key}: {arr_type}: {sub_type} (size: {size})')
