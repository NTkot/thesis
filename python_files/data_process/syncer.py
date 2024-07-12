from .file_wrapper import FileWrapper
from .signal_tools import signal_manip
from .utils import get_by_path
from .segment import Segment, Vector3, Gps, Time

import numpy as np

class Syncer:
    @staticmethod
    def transform(wrapped_data: FileWrapper, imu_topic = '/imu/calib', gps_topic = '/gps/gprmc', interp_method = 'linear') -> Segment:
        imu_abs_time = get_by_path(wrapped_data.data, [imu_topic, 'time_ns'])
        gps_abs_time = get_by_path(wrapped_data.data, [gps_topic, 'time_ns'])
        imu_rel_time = get_by_path(wrapped_data.data, [imu_topic, 'time'])
        gps_rel_time = get_by_path(wrapped_data.data, [gps_topic, 'time'])
        imu_sample_rate = 1 / np.average(imu_rel_time[1:len(imu_rel_time)] - imu_rel_time[0:len(imu_rel_time)-1])
        gps_sample_rate = 1 / np.average(gps_rel_time[1:len(gps_rel_time)] - gps_rel_time[0:len(gps_rel_time)-1])
        # print(f'IMU size: {len(imu_abs_time)} (Sample rate: {imu_sample_rate:.3f}Hz)')
        # print(f'GPS size: {len(gps_abs_time)} (Sample rate: {gps_sample_rate:.3f}Hz)')

        maximum_start_t = max(imu_abs_time[0],  gps_abs_time[0])
        minimum_end_t   = min(imu_abs_time[-1], gps_abs_time[-1])

        imu_slice_idx = signal_manip.find_time_slice_idx(imu_abs_time, [maximum_start_t, minimum_end_t], True)
        gps_slice_idx = signal_manip.find_time_slice_idx(gps_abs_time, [maximum_start_t, minimum_end_t], True)
        # print(f'IMU idx: {imu_slice_idx}')
        # print(f'GPS idx: {gps_slice_idx}')
        
        rel_time_range = [min(imu_rel_time[imu_slice_idx[0]], gps_rel_time[gps_slice_idx[0]]),
                          max(imu_rel_time[imu_slice_idx[1]], gps_rel_time[gps_slice_idx[1]])]
        rel_time = np.arange(rel_time_range[0], rel_time_range[-1],  1 / max(imu_sample_rate, gps_sample_rate), dtype=float)
        # abs_time_range = [min(imu_abs_time[imu_slice_idx[0]], gps_abs_time[gps_slice_idx[0]]), 
        #                   max(imu_abs_time[imu_slice_idx[1]], gps_abs_time[gps_slice_idx[1]])]
        # abs_time = np.arange(abs_time_range[0], abs_time_range[-1], (1 / max(imu_sample_rate, gps_sample_rate)) * 1e9, dtype=int)
        abs_time = (rel_time * 1e9).astype(int) + wrapped_data.minimum_time
        # print(f'IMU abs_time range:   {imu_abs_time[imu_slice_idx[0]]} - {imu_abs_time[imu_slice_idx[1]]}')
        # print(f'GPS abs_time range:   {gps_abs_time[gps_slice_idx[0]]} - {gps_abs_time[gps_slice_idx[1]]}')
        # print(f'Final abs_time range: {abs_time[0]} - {abs_time[-1]}')
        # print(f'IMU rel_time range:   {imu_rel_time[imu_slice_idx[0]]} - {imu_rel_time[imu_slice_idx[1]]}')
        # print(f'GPS rel_time range:   {gps_rel_time[gps_slice_idx[0]]} - {gps_rel_time[gps_slice_idx[1]]}')
        # print(f'Final rel_time range: {rel_time[0]} - {rel_time[-1]}')

        a_x = get_by_path(wrapped_data.data, [imu_topic, 'linear_acceleration', 'x'])
        a_y = get_by_path(wrapped_data.data, [imu_topic, 'linear_acceleration', 'y'])
        a_z = get_by_path(wrapped_data.data, [imu_topic, 'linear_acceleration', 'z'])
        g_x = get_by_path(wrapped_data.data, [imu_topic, 'angular_velocity', 'x'])
        g_y = get_by_path(wrapped_data.data, [imu_topic, 'angular_velocity', 'y'])
        g_z = get_by_path(wrapped_data.data, [imu_topic, 'angular_velocity', 'z'])

        lon   = get_by_path(wrapped_data.data, [gps_topic, 'longitude_deg'])
        lat   = get_by_path(wrapped_data.data, [gps_topic, 'latitude_deg'])
        speed = get_by_path(wrapped_data.data, [gps_topic, 'ground_speed_kmh'])

        a_x_interp, a_y_interp, a_z_interp, g_x_interp, g_y_interp, g_z_interp = \
            signal_manip.interp(imu_rel_time, [a_x, a_y, a_z, g_x, g_y, g_z], rel_time, interp_method)
        lon_interp, lat_interp, speed_interp = \
            signal_manip.interp(gps_rel_time, [lon, lat, speed], rel_time, interp_method)
        
        return Segment(accel=Vector3(x=a_x_interp, y=a_y_interp, z=a_z_interp),
                       gyro=Vector3(x=g_x_interp, y=g_y_interp, z=g_z_interp),
                       gps=Gps(lon=lon_interp, lat=lat_interp, speed=speed_interp),
                       time=Time(rel=rel_time, abs=abs_time))
