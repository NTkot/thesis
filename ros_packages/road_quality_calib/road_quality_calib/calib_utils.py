import math
import numpy as np
import wmm2020 as wmm
import math
import time
from datetime import datetime as dt
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time


def timestamp2int(msg_stamp: Time) -> int:
    """
        Returns int containing ROS2 msg timestamp in ns
    """
    return msg_stamp.sec * int(1e9) + msg_stamp.nanosec


def decyear(date):
    def since_epoch(date): # returns seconds since epoch
        return time.mktime(date.timetuple())
    s = since_epoch

    year = date.year
    start_of_this_year = dt(year=year, month=1, day=1)
    start_of_next_year = dt(year=year+1, month=1, day=1)

    year_elapsed = s(date) - s(start_of_this_year)
    year_duration = s(start_of_next_year) - s(start_of_this_year)
    fraction = year_elapsed / year_duration

    return date.year + fraction


def rotate_180deg_x(rot: Rotation) -> Rotation:
    mat = rot.as_matrix()
    mat[1] = -mat[1]
    mat[2] = -mat[2]
    return Rotation.from_matrix(mat)


def rotate_180deg_z(rot: Rotation) -> Rotation:
    mat = rot.as_matrix()
    mat[0] = -mat[0]
    mat[1] = -mat[1]
    return Rotation.from_matrix(mat)


def moving_average(signal: np.ndarray, window_length: int):
    ret = np.concatenate((np.zeros(window_length-1), signal))
    ret = np.cumsum(ret, dtype=float)
    ret[window_length:] = ret[window_length:] - ret[:-window_length]
    return ret[window_length - 1:] / window_length


class CircularBuffer:
    def __init__(self, size : int, dtype = int):
        self.data  = np.zeros(size).astype(dtype)
        self.dtype = dtype
        self.size  = size

    def insert(self, new_element):
        temp          = self.data[-1]
        self.data     = np.roll(self.data, -1)
        self.data[-1] = self.dtype(new_element)
        return temp

    def clear(self):
        self.data = np.zeros(self.size).astype(self.dtype)


class MovingAverage():
    def __init__(self, size : int):
        self.data = CircularBuffer(size=size, dtype=float)
        self.max_size = size
        self.size = 0

    def insert(self, new_element) -> None:
        self.data.insert(new_element)
        self.size = (self.size + 1) if (self.size < self.max_size) else self.max_size

    def value(self, use_percentile : float = 0.0) -> float:
        if use_percentile > 0.0 and use_percentile < 1.0:
            sorted_data = np.sort(self.data.data)
            data_used = sorted_data[(math.ceil(use_percentile * self.max_size)) :
                                    (math.floor((1-use_percentile) * self.max_size))]
            return np.average(data_used)
        else:
            return np.average(self.data.data)
        
    def active_value(self):
        return np.average(self.data.data[-self.size:])
    
    def clear(self):
        self.data.clear()
        self.size = 0


class MovingAverageAngle():
    """
        Class that calculates average angle readings.
        The reason this needs a different class is 
        because of the discontinuity an angle 
        measurement shows due to warping 
        (0 <-> 360 or -180 <-> 180).
    """
    def __init__(self, size : int):
        self.sin_mv = MovingAverage(size)
        self.cos_mv = MovingAverage(size)
        self.size = size

    def insert_rad(self, new_angle_rad) -> None:
        """
            Insert a new value in radians, defined in any range
        """
        self.sin_mv.insert(math.sin(new_angle_rad))
        self.cos_mv.insert(math.cos(new_angle_rad))

    def insert_deg(self, new_angle_deg) -> None:
        """
            Insert a new value in degrees, defined in any range
        """
        self.insert_rad(new_angle_deg * math.pi / 180.0)

    def value_rad(self) -> float:
        """
            Returns value between -PI and PI
        """
        return math.atan2(self.sin_mv.value(), self.cos_mv.value())

    def value_deg(self) -> float:
        """
            Returns value between -180 and 180
        """
        return self.value_rad() * 180 / math.pi
    
    def clear(self):
        self.sin_mv.clear()
        self.cos_mv.clear()


class Timeseries():
    def __init__(self, max_size : int, time_dtype = float, data_dtype = float):
        self.timedata   = CircularBuffer(size=max_size, dtype=time_dtype)
        self.data       = CircularBuffer(size=max_size, dtype=data_dtype)
        self.time_dtype = time_dtype
        self.data_dtype = data_dtype
        self.max_size   = max_size
        self.size       = 0

    def insert(self, new_timedata_value, new_data_value) -> None:
        self.data.insert(new_data_value)
        self.timedata.insert(new_timedata_value)
        self.size = (self.size + 1) if (self.size < self.max_size) else self.max_size

    def active_timedata(self) -> np.ndarray:
        return self.timedata.data[-self.size:]

    def active_data(self) -> np.ndarray:
        return self.data.data[-self.size:]

    def clear(self) -> None:
        self.data.clear()
        self.timedata.clear()
        self.size = 0


class static_calib():
    @staticmethod
    def accel_to_xy_angles(accel_x, accel_y, accel_z) -> tuple[float, float]:
        x_angle = math.atan2(accel_y, accel_z)
        y_angle = math.atan2(-math.sin(x_angle) * accel_x, accel_y)
        return x_angle, y_angle
    
    @staticmethod
    def xy_angles_to_rot(x_angle, y_angle) -> Rotation:
        sina = math.sin(x_angle)
        cosa = math.cos(x_angle)
        sinb = math.sin(y_angle)
        cosb = math.cos(y_angle)

        rot_mat = [[ cosb, sina*sinb, cosa*sinb],
                   [    0,      cosa,     -sina],
                   [-sinb, cosb*sina, cosa*cosb]]
        
        return Rotation.from_matrix(rot_mat)
    

class dyn_calib():
    @staticmethod
    def gps_magn_to_z_angle(planar_magn_x, planar_magn_y, longitude_deg, latitude_deg, altitude_km, track_deg) -> tuple[float, float]:
        """
            Returns the angle (radians) the magnetometer x-axis (static calib frame) 
            should rotate around z-axis to match with 'track_deg' angle (calib frame).
            Second return is magnetic heading (after applying magnetic declination)
        """
        magn_heading_deg      = math.atan2(planar_magn_y, planar_magn_x) * 180 / math.pi
        magn_declination_deg  = float(wmm.wmm(glats=latitude_deg, glons=longitude_deg, alt_km=altitude_km, yeardec=decyear(dt.today()))["decl"])
        true_magn_heading_deg = magn_heading_deg + magn_declination_deg

        # Convert both angles to [0, 2*PI range]
        track_rad             = (track_deg % 360) * math.pi / 180.0
        true_magn_heading_rad = (true_magn_heading_deg % 360) * math.pi / 180.0

        return true_magn_heading_rad - track_rad, true_magn_heading_rad

    @staticmethod
    def accel_to_z_angle(a_x_planar, a_y_planar) -> tuple[float, float]:
        """
            Returns the angle (radians) that the calib frame should have in 
            relation to static calib frame so its y-axis acceleration is 0.
        """
        z_angle_sin = np.average(np.sin(np.arctan2(-a_y_planar, a_x_planar)))
        z_angle_cos = np.average(np.cos(np.arctan2(-a_y_planar, a_x_planar)))
        z_angle = math.atan2(z_angle_sin, z_angle_cos)

        return z_angle




# if __name__ == '__main__':
    # obj = MovingAverageAngle(5)
    # obj.insert_deg(0)
    # obj.insert_deg(45)
    # obj.insert_deg(-45)
    # obj.insert_deg(-315)
    # obj.insert_deg(315)
    # print(obj.value_deg())

    # obj = MovingAverage(5)
    # obj.insert(-10)
    # obj.insert(-10)
    # print(obj.value())
    # print(obj.active_value())
    # obj.insert(0)
    # obj.insert(0)
    # obj.insert(5)
    # print(obj.value(use_percentile=0.0))

    # obj = Timeseries(max_size=3, time_dtype=int, data_dtype=float)
    # for i in range(1,4):
    #     obj.insert(i, 10*i)
    # print(obj.active_timedata())
    # print(obj.active_data())
    # print(obj.timedata.data)
    # print(obj.data.data)
    # obj.set_data(np.array([10,20,30,40]), np.array([100,200,300,400]))
    # print(obj.active_timedata())
    # print(obj.active_data())
    # print(obj.timedata.data)
    # print(obj.data.data)
    # print(obj.slice(2, 4))