import os


BAG_DB_PATH = '/home/ntkot/theses/ros/bag_db/'


def find_rosbags() -> list[str]:
    rosbag_dirs = []
    for root, _, files in os.walk(BAG_DB_PATH):
        if 'metadata.yaml' in files:
            rosbag_dirs.append(root)
    rosbag_dirs.sort(reverse=True)
    return rosbag_dirs


def find_pickles(path) -> list[str]:
    pickle_dirs = []
    if os.path.exists(path):
        for root, _, files in os.walk(path):
            for file in files:
                if '.pkl' in file:
                    pickle_dirs.append(os.path.join(root, file))
        pickle_dirs.sort(reverse=True)
        return pickle_dirs
    else:
        return []


def number_of_digits(num: int) -> int:
    count = 0
    while num != 0:
        num //= 10
        count += 1
    return count


def prompt_user(choices: list[str], prompt_msg: str) -> list[int]:
    print(prompt_msg)
    max_digits = number_of_digits(len(choices))
    for index, string in enumerate(choices):
        print(f"{str(index).rjust(max_digits, ' ')}) {string}")

    indices = [int(x) for x in input('Enter indices corresponding to your choice: ').split()]
    ge_zero = all(i >= 0 for i in indices)
    lt_max  = all(i < len(choices) for i in indices)
    if not ge_zero:
        raise ValueError('Indices must be greater or equal to 0')
    if not lt_max:
        raise ValueError('Indices must be smaller than the number of total rosbag directories found')
    
    return indices


def prompt_user_yes_no(prompt_msg: str) -> bool:
    user_input = input(prompt_msg + ' (y/n)').strip().lower()
    if user_input == 'yes' or user_input == 'y':
        return True
    else:
        return False


if __name__ == '__main__':
    print(*find_pickles(), sep='\n')
