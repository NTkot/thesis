import os
import math
import argparse
import numpy as np
import compress_pickle
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from data_process.file_wrapper import FileWrapper
from data_process.utils import get_by_path, set_by_path

IMU_RAW_TOPIC = '/imu/raw'
IMU_STATIC_CALIB_TOPIC = '/imu/static_calib'
IMU_CALIB_TOPIC = '/imu/calib'
MAGN_RAW_TOPIC = '/magn/raw'
MAGN_STATIC_CALIB_TOPIC = '/magn/static_calib'
MAGN_CALIB_TOPIC = '/magn/calib'


def move_magnetometer_to_different_topic(data: FileWrapper) -> FileWrapper:
    magn_time    = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'time'])
    magn_time_ns = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'time_ns'])
    m_raw_x      = get_by_path(data.data, [IMU_RAW_TOPIC, 'magnetic_flux_density', 'x'])
    m_raw_y      = get_by_path(data.data, [IMU_RAW_TOPIC, 'magnetic_flux_density', 'y'])
    m_raw_z      = get_by_path(data.data, [IMU_RAW_TOPIC, 'magnetic_flux_density', 'z'])
    m_static_x   = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'magnetic_flux_density', 'x'])
    m_static_y   = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'magnetic_flux_density', 'y'])
    m_static_z   = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'magnetic_flux_density', 'z'])

    data.data[IMU_RAW_TOPIC].pop('magnetic_flux_density')
    data.data[IMU_STATIC_CALIB_TOPIC].pop('magnetic_flux_density')

    set_by_path(data.data, [MAGN_RAW_TOPIC], {})
    set_by_path(data.data, [MAGN_RAW_TOPIC, 'vector'], {})
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC], {})
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector'], {})

    set_by_path(data.data, [MAGN_RAW_TOPIC, 'time'], magn_time)
    set_by_path(data.data, [MAGN_RAW_TOPIC, 'time_ns'], magn_time_ns)
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'time'], magn_time)
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'time_ns'], magn_time_ns)
    set_by_path(data.data, [MAGN_RAW_TOPIC, 'vector', 'x'], m_raw_x)
    set_by_path(data.data, [MAGN_RAW_TOPIC, 'vector', 'y'], m_raw_y)
    set_by_path(data.data, [MAGN_RAW_TOPIC, 'vector', 'z'], m_raw_z)
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'x'], m_static_x)
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'y'], m_static_y)
    set_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'z'], m_static_z)

    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Inject/Replace fully calibrated IMU data to pickle file by setting custom z_angle")

    parser.add_argument("path_to_data", default="", type=str, help="Path to data that can be parsed using data_manip.FileWrapper")
    parser.add_argument("z_angle", type=float, help="Angle to use in degrees to derive rotation matrix from static_calib to calib frame")

    args = parser.parse_args()

    if not os.path.isfile(args.path_to_data):
        raise ValueError("Path to data needs to point to a file")
    data = FileWrapper(args.path_to_data)
    compression = compress_pickle.utils._infer_compression_from_path(args.path_to_data)

    magnetometer_present = [IMU_RAW_TOPIC,  'magnetic_flux_density', 'x'] in data.tree() or \
                           [MAGN_RAW_TOPIC, 'vector', 'x'] in data.tree()

    magnetometer_in_imu_topic = magnetometer_present and ([IMU_RAW_TOPIC, 'magnetic_flux_density', 'x'] in data.tree())
    if magnetometer_in_imu_topic:
        data = move_magnetometer_to_different_topic(data)

    imu_time    = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'time'])
    imu_time_ns = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'time_ns'])
    a_static_x  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'linear_acceleration', 'x'])
    a_static_y  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'linear_acceleration', 'y'])
    a_static_z  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'linear_acceleration', 'z'])
    g_static_x  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'angular_velocity', 'x'])
    g_static_y  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'angular_velocity', 'y'])
    g_static_z  = get_by_path(data.data, [IMU_STATIC_CALIB_TOPIC, 'angular_velocity', 'z'])

    if magnetometer_present:
        magn_time    = get_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'time'])
        magn_time_ns = get_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'time_ns'])
        m_static_x   = get_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'x'])
        m_static_y   = get_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'y'])
        m_static_z   = get_by_path(data.data, [MAGN_STATIC_CALIB_TOPIC, 'vector', 'z'])

    if data.is_static_calibrated():
        cosz = math.cos(args.z_angle * math.pi / 180.0)
        sinz = math.sin(args.z_angle * math.pi / 180.0)
        rot = Rotation.from_matrix([[cosz, -sinz, 0],
                                    [sinz,  cosz, 0],
                                    [   0,     0, 1]])
        
        a_x, a_y, a_z = rot.apply(np.array([a_static_x, a_static_y, a_static_z]).T).T
        g_x, g_y, g_z = rot.apply(np.array([g_static_x, g_static_y, g_static_z]).T).T
        if magnetometer_present:
            m_x, m_y, m_z = rot.apply(np.array([m_static_x, m_static_y, m_static_z]).T).T

        if not data.is_calibrated():
            set_by_path(data.data, [IMU_CALIB_TOPIC], {})
            set_by_path(data.data, [IMU_CALIB_TOPIC,  'linear_acceleration'], {})
            set_by_path(data.data, [IMU_CALIB_TOPIC,  'angular_velocity'], {})

            if magnetometer_present:
                set_by_path(data.data, [MAGN_CALIB_TOPIC], {})
                set_by_path(data.data, [MAGN_CALIB_TOPIC, 'vector'], {})

        set_by_path(data.data, [IMU_CALIB_TOPIC, 'time'], imu_time)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'time_ns'], imu_time_ns)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'linear_acceleration', 'x'], a_x)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'linear_acceleration', 'y'], a_y)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'linear_acceleration', 'z'], a_z)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'angular_velocity', 'x'], g_x)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'angular_velocity', 'y'], g_y)
        set_by_path(data.data, [IMU_CALIB_TOPIC, 'angular_velocity', 'z'], g_z)

        if magnetometer_present:
            set_by_path(data.data, [MAGN_CALIB_TOPIC, 'time'], magn_time)
            set_by_path(data.data, [MAGN_CALIB_TOPIC, 'time_ns'], magn_time_ns)
            set_by_path(data.data, [MAGN_CALIB_TOPIC, 'vector', 'x'], m_x)
            set_by_path(data.data, [MAGN_CALIB_TOPIC, 'vector', 'y'], m_y)
            set_by_path(data.data, [MAGN_CALIB_TOPIC, 'vector', 'z'], m_z)
    else:
        raise ValueError(f"Data are not static calibrated")

    dirpath  = os.path.dirname(args.path_to_data)
    filename = os.path.basename(args.path_to_data).split('.')[0] + "_inj.pkl"
    data.save_to_pickle(os.path.join(dirpath, filename), compression=compression)

    fig = plt.figure(1)
    fig, ax = plt.subplots(num=1, nrows=3, ncols=1, sharex='all')

    ax[0].plot(imu_time, a_x)
    ax[0].grid(which='both', axis='both')
    ax[0].tick_params(labelbottom=True)
    ax[0].set_ylabel('m/s^2')
    ax[0].set_title('Calibrated Acceleration - X')

    ax[1].plot(imu_time, a_y)
    ax[1].grid(which='both', axis='both')
    ax[1].tick_params(labelbottom=True)
    ax[1].set_ylabel('m/s^2')
    ax[1].set_title('Calibrated Acceleration - Y')

    ax[2].plot(imu_time, a_z)
    ax[2].grid(which='both', axis='both')
    ax[2].tick_params(labelbottom=True)
    ax[2].set_ylabel('m/s^2')
    ax[2].set_title('Calibrated Acceleration - Z')

    plt.show()