from road_quality_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from buffers import timeseries_buffer, rate, get_stamp_time_sec

class imu_msg_parser():
    def __init__(self, keep_last: int, rate_filter_size: int = 100):
        self.acc = {'x' : 0.0,
                    'y' : 0.0,
                    'z' : 0.0}
        self.gyro = {'x' : 0.0,
                     'y' : 0.0,
                     'z' : 0.0}
    
        self.acc_history = {'x' : timeseries_buffer(max_size=keep_last, dtype=float),
                            'y' : timeseries_buffer(max_size=keep_last, dtype=float),
                            'z' : timeseries_buffer(max_size=keep_last, dtype=float)}
        self.gyro_history = {'x' : timeseries_buffer(max_size=keep_last, dtype=float),
                             'y' : timeseries_buffer(max_size=keep_last, dtype=float),
                             'z' : timeseries_buffer(max_size=keep_last, dtype=float)}
        
        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Imu):
        self.rate.insert_stamp(msg.header.stamp)
        
        self.acc['x'] = msg.linear_acceleration.x
        self.acc['y'] = msg.linear_acceleration.y
        self.acc['z'] = msg.linear_acceleration.z

        self.gyro['x'] = msg.angular_velocity.x
        self.gyro['y'] = msg.angular_velocity.y
        self.gyro['z'] = msg.angular_velocity.z

        current_time = get_stamp_time_sec(msg.header.stamp)
        self.acc_history['x'].insert(current_time, msg.linear_acceleration.x)
        self.acc_history['y'].insert(current_time, msg.linear_acceleration.y)
        self.acc_history['z'].insert(current_time, msg.linear_acceleration.z)

        self.gyro_history['x'].insert(current_time, msg.angular_velocity.x)
        self.gyro_history['y'].insert(current_time, msg.angular_velocity.y)
        self.gyro_history['z'].insert(current_time, msg.angular_velocity.z)


class magn_msg_parser():
    def __init__(self, keep_last: int, rate_filter_size: int = 100):
        self.magn = {'x' : 0.0,
                     'y' : 0.0,
                     'z' : 0.0}
        
        self.magn_history = {'x' : timeseries_buffer(max_size=keep_last, dtype=float),
                             'y' : timeseries_buffer(max_size=keep_last, dtype=float),
                             'z' : timeseries_buffer(max_size=keep_last, dtype=float)}
        
        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Vector3Stamped):
        self.rate.insert_stamp(msg.header.stamp)

        self.magn['x'] = msg.vector.x * 1e6
        self.magn['y'] = msg.vector.y * 1e6
        self.magn['z'] = msg.vector.z * 1e6

        current_time = get_stamp_time_sec(msg.header.stamp)

        self.magn_history['x'].insert(current_time, msg.vector.x * 1e6)
        self.magn_history['y'].insert(current_time, msg.vector.y * 1e6)
        self.magn_history['z'].insert(current_time, msg.vector.z * 1e6)
