from road_quality_msgs.msg import Gpgga, Gpgsa, Gpgsv, Gprmc, Pcd11
from buffers import timeseries_buffer, rate, get_stamp_time_sec

class gprmc_msg_parser():
    def __init__(self, keep_last: int, rate_filter_size: int = 10):
        self.datetime = ''

        self.longitude_deg = 0.0
        self.longitude_deg_history = timeseries_buffer(max_size=keep_last, dtype=float)
        self.latitude_deg = 0.0
        self.latitude_deg_history  = timeseries_buffer(max_size=keep_last, dtype=float)
        self.longitude_dir = ''
        self.latitude_dir  = ''

        self.ground_speed = 0.0
        self.track_deg    = 0.0
        self.mag_var_deg  = 0.0
        self.mag_var_dir  = ''

        self.pos_valid = False

        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg : Gprmc):
        self.rate.insert_stamp(msg.header.stamp)
        
        self.pos_valid = msg.pos_valid

        if self.pos_valid:
            self.datetime = f'{msg.gps_time.hours:02d}:{msg.gps_time.minutes:02d}:{msg.gps_time.seconds:02d} - '\
                            f'{msg.gps_time.day}/{msg.gps_time.month}/{msg.gps_time.year}'
        else:
            self.datetime = '-'

        self.longitude_deg = msg.longitude_deg
        self.latitude_deg  = msg.latitude_deg

        if  (msg.longitude_dir == msg.LONGITUDE_EAST):
            self.longitude_dir = 'E'
        elif(msg.longitude_dir == msg.LONGITUDE_WEST):
            self.longitude_dir = 'W'
        else:
            self.longitude_dir = ''

        if  (msg.latitude_dir == msg.LATITUDE_NORTH):
            self.latitude_dir = 'N'
        elif(msg.latitude_dir == msg.LATITUDE_SOUTH):
            self.latitude_dir = 'S'
        else:
            self.latitude_dir = ''

        current_time = get_stamp_time_sec(msg.header.stamp)
        self.longitude_deg_history.insert(current_time, self.longitude_deg)
        self.latitude_deg_history.insert(current_time,  self.latitude_deg)

        self.ground_speed = msg.ground_speed_kmh
        self.track_deg    = msg.track_deg
        self.mag_var_deg  = msg.mag_var_deg

        if  (msg.mag_var_dir == msg.MAG_VAR_EAST):
            self.mag_var_dir = 'E'
        elif(msg.mag_var_dir == msg.MAG_VAR_WEST):
            self.mag_var_dir = 'W'
        else:
            self.mag_var_dir = ''


class gpgga_msg_parser():
    def __init__(self, keep_last: int, rate_filter_size: int = 10):
        self.time = ''

        self.longitude_deg = 0.0
        self.longitude_deg_history = timeseries_buffer(max_size=keep_last, dtype=float)
        self.latitude_deg = 0.0
        self.latitude_deg_history  = timeseries_buffer(max_size=keep_last, dtype=float)
        self.longitude_dir = ''
        self.latitude_dir  = ''

        self.altitude = 0.0
        self.altitude_unit = ''

        self.undulation = 0.0
        self.undulation_unit = ''

        self.n_satellites_used = 0

        self.pos_fix_type = 0

        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Gpgga):
        self.rate.insert_stamp(msg.header.stamp)
        
        self.time = f'{msg.gps_time.hours}:{msg.gps_time.minutes}:{msg.gps_time.seconds}'

        self.longitude_deg = msg.longitude_deg
        self.latitude_deg  = msg.latitude_deg

        if  (msg.longitude_dir == msg.LONGITUDE_EAST):
            self.longitude_dir = 'E'
        elif(msg.longitude_dir == msg.LONGITUDE_WEST):
            self.longitude_dir = 'W'
        else:
            self.longitude_dir = ''

        if  (msg.latitude_dir == msg.LATITUDE_NORTH):
            self.latitude_dir = 'N'
        elif(msg.latitude_dir == msg.LATITUDE_SOUTH):
            self.latitude_dir = 'S'
        else:
            self.latitude_dir = ''

        current_time = get_stamp_time_sec(msg.header.stamp)
        self.longitude_deg_history.insert(current_time, self.longitude_deg)
        self.latitude_deg_history.insert(current_time,  self.latitude_deg)

        self.altitude = msg.altitude
        self.altitude_unit = chr(msg.altitude_unit)

        self.undulation = msg.undulation
        self.undulation_unit = chr(msg.undulation_unit)

        self.n_satellites_used = msg.n_satellites

        self.pos_fix_type = msg.position_fix_type


class gpgsa_msg_parser():
    def __init__(self, rate_filter_size: int = 10):
        self.mode = ''

        self.fix_type = ''

        self.sats_used_ids = [-1] * 12

        self.pdop = 0.0
        self.hdop = 0.0
        self.vdop = 0.0

        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Gpgsa):
        self.rate.insert_stamp(msg.header.stamp)
        
        if  (msg.mode == msg.MODE_AUTOMATIC):
            self.mode = 'Automatic'
        elif(msg.mode == msg.MODE_MANUAL):
            self.mode = 'Manual'
        else:
            self.mode = 'unknown'

        if  (msg.fix_type == msg.FIX_TYPE_NOT_AVAILABLE):
            self.fix_type = 'Unavailable'
        elif(msg.fix_type == msg.FIX_TYPE_2D):
            self.fix_type = '2D'
        elif(msg.fix_type == msg.FIX_TYPE_3D):
            self.fix_type = '3D'
        else:
            self.fix_type = 'unknown'

        for i in range(12):
            self.sats_used_ids[i] = msg.sat_ids[i]

        self.pdop = msg.pdop
        self.hdop = msg.hdop
        self.vdop = msg.vdop


class gpgsv_msg_parser():
    def __init__(self, rate_filter_size: int = 12):
        self.n_sentences = 0
        self.current_sentence = 0

        self.n_satellites_view = 0

        self.sat_view_prn = [-1] * 4
        self.sat_view_elevation_deg = [0.0] * 4
        self.sat_view_azimuth_deg = [0.0] * 4
        self.sat_view_snr = [0.0] * 4

        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Gpgsv):
        self.rate.insert_stamp(msg.header.stamp)
        
        self.n_sentences = msg.n_sentences
        self.current_sentence = msg.current_sentence

        self.n_satellites_view = msg.n_satellites_view

        for i in range(4):
            self.sat_view_prn[i] = msg.sat_prn[i]
            self.sat_view_elevation_deg[i] = msg.sat_elevation_deg[i]
            self.sat_view_azimuth_deg[i] = msg.sat_azimuth_deg[i]
            self.sat_view_snr[i] = msg.sat_snr[i]


class pcd11_msg_parser():
    def __init__(self, rate_filter_size: int = 10):
        self.antenna_status = 0

        self.rate = rate(rate_filter_size)

    def parse_msg(self, msg: Pcd11):
        self.rate.insert_stamp(msg.header.stamp)

        self.antenna_status = msg.antenna_status
