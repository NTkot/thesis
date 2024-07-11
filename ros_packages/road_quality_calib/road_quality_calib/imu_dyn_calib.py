import os
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import wiener
from time import sleep
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster

from road_quality_msgs.msg import Imu, Gprmc, Gpgga, PinState
from road_quality_msgs.srv import GetImuCalib
from geometry_msgs.msg import TransformStamped, Vector3Stamped

from calib_utils import MovingAverage, MovingAverageAngle, Timeseries, dyn_calib
from calib_utils import timestamp2int, moving_average, rotate_180deg_z


class ImuDynCalibCore(Node):
    def var_init(self):
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = 'imu'
        self.tf_msg.child_frame_id = 'imu_calib'
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        

    def ros_init(self):
        if not self.static_calib_done():
            self.get_logger().error('IMU Static calibration not detected. Run it first...')
            raise SystemExit
        self.get_logger().info('Got IMU Static calibration')
        
        self.tf_broadcaster = TransformBroadcaster(self)


    def static_calib_done(self) -> bool:
        self.imu_calib_client = self.create_client(GetImuCalib, 'get_imu_calib')
        
        self.get_logger().info('Waiting for GetImuCalib service to be created')
        while not self.imu_calib_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('GetImuCalib service unavailable, exiting...')
            raise SystemExit

        self.request = GetImuCalib.Request()
        self.request.static_calib_only = True
        self.get_logger().info('Calling GetImuCalib service to get IMU Static calibration')
        self.future = self.imu_calib_client.call_async(self.request)

        self.get_logger().info('Waiting for GetImuCalib service response')
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        self.static_calib_from_imu = Rotation.from_quat([self.response.calib.transform.rotation.x,
                                                         self.response.calib.transform.rotation.y,
                                                         self.response.calib.transform.rotation.z,
                                                         self.response.calib.transform.rotation.w])
        
        self.get_logger().info(f'GetImuCalib service response is {self.response.calib_valid}')
        return self.response.calib_valid




class ImuDynCalibMagn(ImuDynCalibCore):
    def __init__(self):
        super(ImuDynCalibCore, self).__init__('imu_dyn_calib')

        self.param_init()

        self.var_init()

        self.ros_init()

        self.get_logger().info('Dynamic IMU calibration (GPS-Magnetometer method) initialized with:\n'
                              f'Magn window size = {self.magn_window_size}\n'
                              f'GPS window size  = {self.gps_window_size}')


    def param_init(self):
        self.declare_parameter('magn_window_size',    400)
        self.declare_parameter('gps_window_size',     20)
        self.declare_parameter('speed_threshold_kmh', 5.0)
        self.declare_parameter('led_pin',             5)


    def var_init(self):
        super().var_init()
        
        self.magn_window_size = self.get_parameter('magn_window_size').get_parameter_value().integer_value
        self.gps_window_size  = self.get_parameter('gps_window_size').get_parameter_value().integer_value
        self.speed_threshold  = self.get_parameter('speed_threshold_kmh').get_parameter_value().double_value

        self.magn_filter = {'x' : MovingAverage(self.magn_window_size),
                            'y' : MovingAverage(self.magn_window_size),
                            'z' : MovingAverage(self.magn_window_size)}
        self.rmc_filter  = {'lon_deg'   : MovingAverageAngle(self.gps_window_size),
                            'lat_deg'   : MovingAverageAngle(self.gps_window_size),
                            'track_deg' : MovingAverageAngle(self.gps_window_size)}
        self.gga_filter  = {'alt_km'    : MovingAverage(self.gps_window_size)}

        self.speed = 0
        self.threshold_surpassed = False

        self.led_msg = PinState()
        self.led_msg.pin = self.get_parameter('led_pin').get_parameter_value().integer_value
        
        self.magn_msgs_received = 0
        self.rmc_msgs_received  = 0
        self.gga_msgs_received  = 0

        self.calibrating = False


    def ros_init(self):
        super().ros_init()
        
        self.rmc_sub  = self.create_subscription(Gprmc, '/gps/gprmc', 
                                                self.rmc_magn_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)
        self.gga_sub  = self.create_subscription(Gpgga, '/gps/gpgga', 
                                                self.gga_magn_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)
        self.magn_sub = self.create_subscription(Vector3Stamped, '/magn/static_calib', 
                                                self.imu_static_magn_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)
        self.led_pub  = self.create_publisher(PinState, f'/gpio/outputs/pin{self.led_msg.pin:d}',
                                             QoSPresetProfiles.SERVICES_DEFAULT.value)


    def imu_static_magn_callback(self, msg: Vector3Stamped):
        if(not self.stop_gathering_msgs and self.start_gathering_msgs):
            self.magn_msgs_received += 1
            self.magn_filter['x'].insert(msg.vector.x * 1e6)
            self.magn_filter['y'].insert(msg.vector.y * 1e6)
            self.magn_filter['z'].insert(msg.vector.z * 1e6)
        elif (not self.calibrating):
            self.calibrating = True
            self.perform_calib()
        else:
            pass


    def rmc_magn_callback(self, msg : Gprmc):
        self.speed = msg.ground_speed_kmh

        if self.start_gathering_msgs:
            self.led_msg.state = 0
        else:
            self.led_msg.state = 1
        self.led_msg.header.stamp = self.get_clock().now().to_msg()
        self.led_pub.publish(self.led_msg)

        if(not self.stop_gathering_msgs and self.start_gathering_msgs):
            if not msg.pos_valid:
                self.get_logger().error('Received invalid GPS message. Dynamic calibration will not take place')
                raise SystemExit

            self.rmc_msgs_received += 1
            self.rmc_filter['lon_deg'].insert_deg(msg.longitude_deg)
            self.rmc_filter['lat_deg'].insert_deg(msg.latitude_deg)
            self.rmc_filter['track_deg'].insert_deg(msg.track_deg)
        else:
            pass


    def gga_magn_callback(self, msg : Gpgga):
        if(not self.stop_gathering_msgs and self.start_gathering_msgs):
            self.gga_msgs_received += 1
            self.gga_filter['alt_km'].insert(msg.altitude / 1000)
        else:
            pass


    @property
    def start_gathering_msgs(self) -> bool:
        speed_good = (self.speed >= self.speed_threshold)
        if speed_good:
            self.threshold_surpassed = True
        return (speed_good or self.threshold_surpassed)


    @property
    def stop_gathering_msgs(self) -> bool:
        return ((self.rmc_msgs_received  >  (1.25 * self.gps_window_size)) and
                (self.gga_msgs_received  >= self.gps_window_size)          and
                (self.magn_msgs_received >= self.magn_window_size))


    def perform_calib(self):
        self.tf_msg.header.stamp = self.get_clock().now().to_msg()

        magn_x = self.magn_filter['x'].value(use_percentile=0.1)
        magn_y = self.magn_filter['y'].value(use_percentile=0.1)
        magn_z = self.magn_filter['z'].value(use_percentile=0.1)
        
        lon_deg   = self.rmc_filter['lon_deg'].value_deg()
        lat_deg   = self.rmc_filter['lat_deg'].value_deg()
        track_deg = self.rmc_filter['track_deg'].value_deg()

        alt_km = self.gga_filter['alt_km'].value()

        z_angle, magn_heading = dyn_calib.gps_magn_to_z_angle(planar_magn_x=magn_x, 
                                                              planar_magn_y=magn_y, 
                                                              longitude_deg=lon_deg,
                                                              latitude_deg=lat_deg,
                                                              altitude_km=alt_km,
                                                              track_deg=track_deg)
        
        sinz = math.sin(z_angle)
        cosz = math.cos(z_angle)
        
        calib_from_static_calib = Rotation.from_matrix([[ cosz, sinz, 0],
                                                        [-sinz, cosz, 0],
                                                        [    0,    0, 1]])

        calib_from_imu = calib_from_static_calib * self.static_calib_from_imu

        quat = calib_from_imu.as_quat()
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.get_logger().info('Dynamic IMU calibration done, found:\n'
                              f'z_angle      = {z_angle * 180 / math.pi:.3f}°\n'
                              f'magn_heading = {magn_heading * 180 / math.pi:.3f}°')
        
        rot_mat = calib_from_static_calib.as_matrix()
        self.get_logger().info('Rotation matrix (calib <- static_calib):\n'
                              f'[{rot_mat[0][0]:.4f} {rot_mat[0][1]:.4f} {rot_mat[0][2]:.4f}\n'
                              f' {rot_mat[1][0]:.4f} {rot_mat[1][1]:.4f} {rot_mat[1][2]:.4f}\n'
                              f' {rot_mat[2][0]:.4f} {rot_mat[2][1]:.4f} {rot_mat[2][2]:.4f}]')
        
        self.get_logger().info('Magnetometer values used:\n'
                              f'B_x: {magn_x:.1f} uT\n'
                              f'B_y: {magn_y:.1f} uT\n'
                              f'B_z: {magn_z:.1f} uT\n\n'
                               'GPS values used:\n'
                              f'Longitude:   {lon_deg:.7f}°\n'
                              f'Latitude:    {lat_deg:.7f}°\n'
                              f'Altitude:    {alt_km*1000:.2f}m\n'
                              f'Track angle: {track_deg:.2f}°')
        
        self.tf_broadcaster.sendTransform(self.tf_msg)

        raise SystemExit




class ImuDynCalibBrake(ImuDynCalibCore):
    def __init__(self):
        super(ImuDynCalibCore, self).__init__('imu_dyn_calib')

        self.param_init()

        self.var_init()

        self.ros_init()

        self.get_logger().info(('(OFFLINE MODE) ' if self.offline_mode else '') +
                                'Dynamic IMU calibration (braking method) initialized with:\n'
                                'Buffer sizes:\n'
                               f'  accel: {self.accel_buffer_size}\n'
                               f'  gps:   {self.gps_buffer_size}\n'
                                'Brake detect:\n'
                               f'  GPS samples threshold: {self.brake_detect_samples_threshold}\n'
                               f'  GPS decel threshold:   {self.brake_detect_gps_decel_threshold:.1f} m/s^2\n'
                               f'  IMU decel threshold:   {self.brake_detect_imu_decel_threshold:.1f} m/s^2\n'
                               f'  Track range threshold: {self.brake_detect_track_range_threshold:.1f}°\n'
                                'IMU filter:\n'
                               f'  Wiener size:         {self.imu_filter_wiener_size}\n'
                               f'  Moving average size: {self.imu_filter_mv_size}')


    def param_init(self):
        self.declare_parameter('accel_buffer_size', 1200)
        self.declare_parameter('gps_buffer_size', 60)
        self.declare_parameter('led_pin', 5)
        self.declare_parameter('brake_detect_gps_decel_threshold', -2.0)
        self.declare_parameter('brake_detect_imu_decel_threshold', 2.0)
        self.declare_parameter('brake_detect_track_range_threshold', 2.0)
        self.declare_parameter('brake_detect_samples_threshold', 5)
        self.declare_parameter('imu_filter_wiener_size', 101)
        self.declare_parameter('imu_filter_mv_size', 6)
        self.declare_parameter('offline_mode', False)


    def var_init(self):
        super().var_init()

        self.accel_buffer_size                  = self.get_parameter('accel_buffer_size').get_parameter_value().integer_value
        self.gps_buffer_size                    = self.get_parameter('gps_buffer_size').get_parameter_value().integer_value
        self.brake_detect_gps_decel_threshold   = self.get_parameter('brake_detect_gps_decel_threshold').get_parameter_value().double_value
        self.brake_detect_imu_decel_threshold   = self.get_parameter('brake_detect_imu_decel_threshold').get_parameter_value().double_value
        self.brake_detect_track_range_threshold = self.get_parameter('brake_detect_track_range_threshold').get_parameter_value().double_value
        self.brake_detect_samples_threshold     = self.get_parameter('brake_detect_samples_threshold').get_parameter_value().integer_value
        self.imu_filter_wiener_size             = self.get_parameter('imu_filter_wiener_size').get_parameter_value().integer_value
        self.imu_filter_mv_size                 = self.get_parameter('imu_filter_mv_size').get_parameter_value().integer_value
        self.offline_mode                       = self.get_parameter('offline_mode').get_parameter_value().bool_value

        self.accel_buffer = {'x' : Timeseries(max_size = self.accel_buffer_size, time_dtype=int, data_dtype=float),
                             'y' : Timeseries(max_size = self.accel_buffer_size, time_dtype=int, data_dtype=float)}
        
        self.rmc_buffer = {'speed': Timeseries(max_size = self.gps_buffer_size, time_dtype=int, data_dtype=float),
                           'accel': Timeseries(max_size = self.gps_buffer_size, time_dtype=int, data_dtype=float),
                           'track': Timeseries(max_size = self.gps_buffer_size, time_dtype=int, data_dtype=float),
                           'sample_rate': MovingAverage(size=10)}

        self.led_msg = PinState()
        self.led_msg.pin = self.get_parameter('led_pin').get_parameter_value().integer_value

        self.stop_gathering_rmc_msgs = False
        self.stop_gathering_imu_msgs = False
        self.imu_msgs_received = 0

        self.last_imu_time = int(0)
        self.last_rmc_time = int(0)

        self.brake_vars = {'gps_start': 0,
                           'gps_end': 0,
                           'imu_start': 0,
                           'imu_end': 0,
                           'braking': False,
                           'counter': 0,
                           'track_angles': []}


    def ros_init(self):
        if not self.offline_mode:
            super().ros_init()
        else:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.static_calib_from_imu = Rotation.from_matrix([[1, 0, 0],
                                                               [0, 1, 0],
                                                               [0, 0, 1]])

        self.rmc_sub = self.create_subscription(Gprmc, '/gps/gprmc', 
                                                self.rmc_brake_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)
        self.imu_sub = self.create_subscription(Imu, '/imu/static_calib', 
                                                self.imu_static_brake_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)
        self.led_pub = self.create_publisher(PinState, f'/gpio/outputs/pin{self.led_msg.pin:d}',
                                             QoSPresetProfiles.SERVICES_DEFAULT.value)


    def rmc_brake_callback(self, msg: Gprmc):
        if not self.stop_gathering_rmc_msgs:
            if not msg.pos_valid:
                self.get_logger().error('Received invalid GPS message. Dynamic calibration will not take place')
                raise SystemExit
            
            msg_time_ns = timestamp2int(msg.header.stamp)
            self.rmc_buffer['track'].insert(msg_time_ns, msg.track_deg)
            self.rmc_buffer['speed'].insert(msg_time_ns, msg.ground_speed_kmh / 3.6)

            if self.last_rmc_time != 0:
                dt = float(msg_time_ns - self.last_rmc_time) / 1e9
                self.rmc_buffer['sample_rate'].insert(1/dt)

                self.rmc_buffer['accel'].insert(msg_time_ns, (self.rmc_buffer['speed'].data.data[-1] - \
                                                              self.rmc_buffer['speed'].data.data[-2]) * self.rmc_buffer['sample_rate'].active_value())
                self.detect_brake()
                self.get_logger().info(f'Checking for possible brake ({self.brake_vars["counter"]})')
            else:
                self.rmc_buffer['accel'].insert(msg_time_ns, 0)
            
            self.last_rmc_time = msg_time_ns


    def imu_static_brake_callback(self, msg: Imu):
        if not self.stop_gathering_imu_msgs:
            msg_time_ns = timestamp2int(msg.header.stamp)

            self.accel_buffer['x'].insert(msg_time_ns, msg.linear_acceleration.x)
            self.accel_buffer['y'].insert(msg_time_ns, msg.linear_acceleration.y)

            self.last_imu_time = msg_time_ns
            self.imu_msgs_received += 1

            if self.imu_msgs_received == self.imu_filter_wiener_size:
                self.get_logger().info('Gathered IMU samples needed for correct wiener filter application')
                self.turn_on_led()

            if self.stop_gathering_rmc_msgs and (msg_time_ns - self.last_rmc_time > 1e9):
                self.get_logger().info('Gathered IMU samples needed for calib')
                self.stop_gathering_imu_msgs = True
                self.turn_off_led()
                self.perform_calib()


    def detect_brake(self):
        new_accel = self.rmc_buffer['accel'].active_data()[-1]
        new_track = self.rmc_buffer['track'].active_data()[-1]

        if new_accel <= self.brake_detect_gps_decel_threshold:
            self.brake_vars['counter'] += 1
            self.brake_vars['track_angles'].append(new_track)
        else:
            if self.brake_vars['braking']:
                self.brake_vars['braking'] = False
                self.stop_gathering_rmc_msgs = True

                track_angle_range = max(self.brake_vars['track_angles']) - min(self.brake_vars['track_angles'])
                track_angle_range = abs((track_angle_range + 180) % 360 - 180)
                if track_angle_range < self.brake_detect_track_range_threshold:
                    self.brake_vars['gps_start'] = self.rmc_buffer['accel'].active_timedata()[-1-self.brake_vars['counter']]
                    self.brake_vars['gps_end']   = self.rmc_buffer['accel'].active_timedata()[-2]
                    self.get_logger().info('Detected straight line braking from GPS:\n'
                                          f'  Total active data:  {self.rmc_buffer["speed"].size}\n'
                                          f'  Total braking data: {self.brake_vars["counter"]}\n'
                                          f'  Braking duration:   {float(self.brake_vars["gps_end"] - self.brake_vars["gps_start"]) / 1e9:.3f}\n'
                                          f'  Track angle range:  {track_angle_range:.2f}°')
                else:
                    self.get_logger().error(f'Braking detected, but it was not within straight line threshold (range: {track_angle_range}°). Exiting...')
                    self.blink_led(times=10, delay_ms=250)
                    raise SystemExit

            self.brake_vars['counter'] = 0
            self.brake_vars['track_angles'] = []

        if self.brake_vars['counter'] >= self.brake_detect_samples_threshold and not self.brake_vars['braking']:
            self.brake_vars['braking'] = True


    def time_correlate_gps_imu_braking(self):
        a_x = moving_average(wiener(self.accel_buffer['x'].active_data(), self.imu_filter_wiener_size), self.imu_filter_mv_size)
        a_y = moving_average(wiener(self.accel_buffer['y'].active_data(), self.imu_filter_wiener_size), self.imu_filter_mv_size)
        a_planar = np.sqrt(a_x ** 2 + a_y ** 2)
        imu_time = self.accel_buffer['x'].active_timedata()

        gps_brake_start_t = self.brake_vars['gps_start']
        gps_brake_end_t   = self.brake_vars['gps_end']

        imu_slice_start_idx = min(range(len(imu_time)), key=lambda i: abs(imu_time[i] - (gps_brake_start_t - 5e8)))
        imu_slice_end_idx   = min(range(len(imu_time)), key=lambda i: abs(imu_time[i] - (gps_brake_end_t   + 3e8)))
        a_planar_slice      = a_planar[imu_slice_start_idx:(imu_slice_end_idx+1)]

        if np.where(a_planar_slice > self.brake_detect_imu_decel_threshold)[0].size == 0:
            self.get_logger().error('Could not time-correlate GPS and IMU deceleration')
            raise SystemExit
        # Find first occurance where planar acceleration surpasses brake_detect_imu_decel_threshold
        imu_brake_start_idx = np.where(a_planar_slice > self.brake_detect_imu_decel_threshold)[0][0]
        # Find last occurance where planar acceleration surpasses brake_detect_imu_decel_threshold
        imu_brake_end_idx   = len(a_planar_slice) - np.where(np.flip(a_planar_slice) > self.brake_detect_imu_decel_threshold)[0][0] - 1

        brake_start_idx = imu_brake_start_idx + imu_slice_start_idx
        brake_end_idx   = imu_brake_end_idx   + imu_slice_start_idx

        a_x_braking      = a_x[brake_start_idx:(brake_end_idx+1)]
        a_y_braking      = a_y[brake_start_idx:(brake_end_idx+1)]
        a_planar_braking = a_planar[brake_start_idx:(brake_end_idx+1)]
        self.brake_vars['imu_start'] = imu_time[imu_brake_start_idx + imu_slice_start_idx]
        self.brake_vars['imu_end']   = imu_time[imu_brake_end_idx   + imu_slice_start_idx]

        self.get_logger().info('IMU Brake stats:\n'
                              f'  Total active data:  {len(a_x)}\n'
                              f'  Total braking data: {len(a_x_braking)}\n'
                              f'  Braking duration:   {float(self.brake_vars["imu_end"] - self.brake_vars["imu_start"]) / 1e9:.3f}\n'
                              f'  Braking indices:    {imu_brake_start_idx + imu_slice_start_idx}-{imu_brake_end_idx + imu_slice_start_idx}\n'
                              f'  x-axis:\n'
                              f'    min: {min(a_x_braking)}\n'
                              f'    max: {max(a_x_braking)}\n'
                              f'    avg: {np.average(a_x_braking)}\n'
                              f'  y-axis:\n'
                              f'    min: {min(a_y_braking)}\n'
                              f'    max: {max(a_y_braking)}\n'
                              f'    avg: {np.average(a_y_braking)}\n'
                              f'  total-planar:\n'
                              f'    min: {min(a_planar_braking)}\n'
                              f'    max: {max(a_planar_braking)}\n'
                              f'    avg: {np.average(a_planar_braking)}')

        return a_x_braking, a_y_braking


    def perform_calib(self):
        a_x, a_y = self.time_correlate_gps_imu_braking()

        z_angle = dyn_calib.accel_to_z_angle(a_x, a_y)
        sinz = math.sin(z_angle)
        cosz = math.cos(z_angle)
        
        calib_from_static_calib = Rotation.from_matrix([[cosz, -sinz, 0],
                                                        [sinz,  cosz, 0],
                                                        [   0,     0, 1]])
        
        a_x_transformed = calib_from_static_calib.apply(np.array([np.average(a_x), np.average(a_y), 0]))[0]
        if(a_x_transformed > 0):
            calib_from_static_calib = rotate_180deg_z(calib_from_static_calib)
            self.get_logger().info('Rotating around Z by 180° to make a_x_calib negative during braking')

        calib_from_imu = calib_from_static_calib * self.static_calib_from_imu

        quat = calib_from_imu.as_quat()
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.get_logger().info('Dynamic IMU calibration done, found:\n'
                              f'z_angle = {z_angle * 180 / math.pi:.3f}°' +
                             (f' (final = {math.atan2(math.sin(z_angle + math.pi), math.cos(z_angle + math.pi)) * 180 / math.pi:.3f}°)' if a_x_transformed > 0 else ''))
        
        rot_mat = calib_from_static_calib.as_matrix()
        self.get_logger().info('Rotation matrix (calib <- static_calib):\n'
                              f'[{rot_mat[0][0]:.4f} {rot_mat[0][1]:.4f} {rot_mat[0][2]:.4f}\n'
                              f' {rot_mat[1][0]:.4f} {rot_mat[1][1]:.4f} {rot_mat[1][2]:.4f}\n'
                              f' {rot_mat[2][0]:.4f} {rot_mat[2][1]:.4f} {rot_mat[2][2]:.4f}]')
        
        self.tf_broadcaster.sendTransform(self.tf_msg)

        raise SystemExit


    def turn_on_led(self):
        self.led_msg.header.stamp = self.get_clock().now().to_msg()
        self.led_msg.state = True
        self.led_pub.publish(self.led_msg)


    def turn_off_led(self):
        self.led_msg.header.stamp = self.get_clock().now().to_msg()
        self.led_msg.state = False
        self.led_pub.publish(self.led_msg)


    def blink_led(self, times: int, delay_ms: int):
        for i in range(times):
            self.turn_on_led()
            sleep(float(delay_ms) / 1000)
            self.turn_off_led()
            sleep(float(delay_ms) / 1000)




class ImuDynCalibUserValues(ImuDynCalibCore):
    def __init__(self):
        super(ImuDynCalibCore, self).__init__('imu_dyn_calib')

        self.param_init()

        self.var_init()

        self.ros_init()

        self.get_logger().info('Dynamic IMU calibration initialized with user values')

        self.publish_user_tf()

    
    def param_init(self):
        self.declare_parameter('z_angle', 0.0)
        self.declare_parameter('calib_file', '')


    def var_init(self):
        super().var_init()

        self.z_angle_param = self.get_parameter('z_angle').get_parameter_value().double_value
        self.calib_file = self.get_parameter('calib_file').get_parameter_value().string_value


    def ros_init(self):
        super().ros_init()


    def publish_user_tf(self):
        if self.calib_file and os.path.isfile(self.calib_file):
            with open(self.calib_file) as f:
                calib_file_yaml = yaml.load(f, Loader=yaml.FullLoader)
            z_angle = calib_file_yaml['z_angle']
            self.get_logger().info(f'Using calib file: {self.calib_file}')
        else:
            z_angle = self.z_angle_param
            self.get_logger().info(f'Using passed ROS parameters as angles')
        sinz = math.sin(z_angle)
        cosz = math.cos(z_angle)

        calib_from_static_calib = Rotation.from_matrix([[cosz, -sinz, 0],
                                                        [sinz,  cosz, 0],
                                                        [   0,     0, 1]])

        calib_from_imu = calib_from_static_calib * self.static_calib_from_imu

        quat = calib_from_imu.as_quat()
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.get_logger().info('Dynamic IMU calibration done, found:\n'
                              f'z_angle = {z_angle * 180 / math.pi:.3f}°')
        
        rot_mat = calib_from_static_calib.as_matrix()
        self.get_logger().info('Rotation matrix (calib <- static_calib):\n'
                              f'[{rot_mat[0][0]:.4f} {rot_mat[0][1]:.4f} {rot_mat[0][2]:.4f}\n'
                              f' {rot_mat[1][0]:.4f} {rot_mat[1][1]:.4f} {rot_mat[1][2]:.4f}\n'
                              f' {rot_mat[2][0]:.4f} {rot_mat[2][1]:.4f} {rot_mat[2][2]:.4f}]')
        
        self.tf_broadcaster.sendTransform(self.tf_msg)

        raise SystemExit
