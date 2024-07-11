from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.publisher import Publisher
import tf2_ros
import ament_index_python
from PyQt5 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3Stamped

from interface import Ui_main_window
from gps_parsers import gprmc_msg_parser, gpgga_msg_parser, gpgsa_msg_parser, gpgsv_msg_parser, pcd11_msg_parser
from imu_parsers import imu_msg_parser, magn_msg_parser
from plot_utils import update_timeplot
from buffers import moving_average, autoclear_checker, get_stamp_time_sec

from road_quality_msgs.msg import Imu, Gpgga, Gpgsa, Gpgsv, Gprmc, Pcd11, PinState, PinPwmState

import yaml
import pyqtgraph as pg

class Monitor(Node):
    def __init__(self):
        super().__init__('gui_monitor_node')

        self.load_config()

        self.init_gui()

        self.init_vars()

        # self.init_tf_listener()

        self.init_ros()

        self.organize_widgets_to_arrays()
        self.setup_misc_widgets()
        self.setup_button_widgets()
        self.setup_plot_widgets()
        self.setup_gauge_widgets()


    def load_config(self):
        with open(ament_index_python.get_package_share_directory('road_quality_monitor') + '/monitor.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)


    def init_gui(self):
        self.main_window = QtWidgets.QMainWindow()
        self.ui = Ui_main_window()
        self.ui.setupUi(self.main_window)
        self.main_window.show()
        self.main_window.setWindowIcon(QtGui.QIcon(ament_index_python.get_package_share_directory('road_quality_monitor') + '/road_icon.png'))


    def init_vars(self):
        self.imu_counter = 100
        self.imu_is_published = False
        self.magn_counter = 100
        self.magn_is_published = False

        self.imu_static_calib_counter = 100
        self.imu_static_calib_is_published = False
        self.magn_static_calib_counter = 100
        self.magn_static_calib_is_published = False

        self.imu_calib_counter = 100
        self.imu_calib_is_published = False
        self.magn_calib_counter = 100
        self.magn_calib_is_published = False

        self.camera_fps_filter = moving_average(13)
        self.last_camera_msg_time = get_stamp_time_sec(self.get_clock().now().to_msg())
        for i in range(self.camera_fps_filter.window_length):
            self.camera_fps_filter.insert(30)

        self.active_nodes = {'nucleus' : False, 'gpio': False,
                             'imu' : False, 'gps': False,
                             'camera' : False, 'static_calib' : False,
                             'rosbag_rec' : False, 'dyn_calib' : False}
        self.active_nodes_ui_element = {'nucleus' : self.ui.nucleus_active, 'gpio': self.ui.gpio_active,
                                        'imu' : self.ui.imu_active, 'gps': self.ui.gps_active,
                                        'camera' : self.ui.camera_active, 'static_calib' : self.ui.static_calib_active,
                                        'rosbag_rec' : self.ui.rosbag_rec_active, 'dyn_calib' : self.ui.dyn_calib_active}
        

    def init_tf_listener(self):
        self.tf_buffer   = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)


    def init_ros(self):
        self.gprmc_parser = gprmc_msg_parser(keep_last=int(self.config['gps_keep_last_msgs']))
        self.gprmc_sub = self.create_subscription(Gprmc, '/gps/gprmc', self.gprmc_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.gpgga_parser = gpgga_msg_parser(keep_last=int(self.config['gps_keep_last_msgs']))
        self.gpgga_sub = self.create_subscription(Gpgga, '/gps/gpgga', self.gpgga_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.gpgsa_parser = gpgsa_msg_parser()
        self.gpgsa_sub = self.create_subscription(Gpgsa, '/gps/gpgsa', self.gpgsa_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.gpgsv_parser = gpgsv_msg_parser()
        self.gpgsv_sub = self.create_subscription(Gpgsv, '/gps/gpgsv', self.gpgsv_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.pcd11_parser = pcd11_msg_parser()
        self.pcd11_sub = self.create_subscription(Pcd11, '/gps/pcd11', self.pcd11_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.imu_parser = imu_msg_parser(keep_last=int(self.config['imu_keep_last_msgs']))
        self.imu_sub = self.create_subscription(Imu, '/imu/raw', self.imu_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.imu_static_calib_parser = imu_msg_parser(keep_last=int(self.config['imu_keep_last_msgs']))
        self.imu_static_calib_sub = self.create_subscription(Imu, '/imu/static_calib', self.imu_static_calib_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.imu_calib_parser = imu_msg_parser(keep_last=int(self.config['imu_keep_last_msgs']))
        self.imu_calib_sub = self.create_subscription(Imu, '/imu/calib', self.imu_calib_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.magn_parser = magn_msg_parser(keep_last=int(self.config['magn_keep_last_msgs']))
        self.magn_sub = self.create_subscription(Vector3Stamped, '/magn/raw', self.magn_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.magn_static_calib_parser = magn_msg_parser(keep_last=int(self.config['magn_keep_last_msgs']))
        self.magn_static_calib_sub = self.create_subscription(Vector3Stamped, '/magn/static_calib', self.magn_static_calib_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.magn_calib_parser = magn_msg_parser(keep_last=int(self.config['magn_keep_last_msgs']))
        self.magn_calib_sub = self.create_subscription(Vector3Stamped, '/magn/calib', self.magn_calib_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.camera_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.camera_callback, 2)

        self.ext_button_sub         = self.create_subscription(PinState, '/gpio/inputs/pin' + str(self.config['external_button']['pin']), self.external_button_callback, QoSPresetProfiles.SERVICES_DEFAULT.value)
        self.blue_led_button_pub    = self.create_publisher(PinState,    '/gpio/inputs/pin' + str(self.config['blue_led_button']['pin']),   QoSPresetProfiles.SERVICES_DEFAULT.value)
        self.red_led_button_pub     = self.create_publisher(PinState,    '/gpio/inputs/pin' + str(self.config['red_led_button']['pin']),    QoSPresetProfiles.SERVICES_DEFAULT.value)
        self.yellow_led_button_pub  = self.create_publisher(PinState,    '/gpio/inputs/pin' + str(self.config['yellow_led_button']['pin']), QoSPresetProfiles.SERVICES_DEFAULT.value)
        self.green_led_button_pub   = self.create_publisher(PinState,    '/gpio/inputs/pin' + str(self.config['green_led_button']['pin']),  QoSPresetProfiles.SERVICES_DEFAULT.value)
        self.fan_control_dial_pub   = self.create_publisher(PinPwmState, '/gpio/pwm/pin'    + str(self.config['fan_control_pin']),          QoSPresetProfiles.SERVICES_DEFAULT.value)

        self.imu_calib_check_timer = self.create_timer(0.25, self.check_imu_calib)
        self.active_node_timer = self.create_timer(0.5, self.check_active_nodes)


    def organize_widgets_to_arrays(self):
        self.ui.gps_id_sat_values = [self.ui.gps_id_sat0_value, self.ui.gps_id_sat1_value,  self.ui.gps_id_sat2_value,
                                     self.ui.gps_id_sat3_value, self.ui.gps_id_sat4_value,  self.ui.gps_id_sat5_value,
                                     self.ui.gps_id_sat6_value, self.ui.gps_id_sat7_value,  self.ui.gps_id_sat8_value,
                                     self.ui.gps_id_sat9_value, self.ui.gps_id_sat10_value, self.ui.gps_id_sat11_value,]
        
        self.ui.gps_prn_sat_values  = [[self.ui.gps_sat0_prn_value,  self.ui.gps_sat1_prn_value,
                                        self.ui.gps_sat2_prn_value,  self.ui.gps_sat3_prn_value],
                                       [self.ui.gps_sat4_prn_value,  self.ui.gps_sat5_prn_value,
                                        self.ui.gps_sat6_prn_value,  self.ui.gps_sat7_prn_value],
                                       [self.ui.gps_sat8_prn_value,  self.ui.gps_sat9_prn_value,
                                        self.ui.gps_sat10_prn_value, self.ui.gps_sat11_prn_value]]
        
        self.ui.gps_elev_sat_values = [[self.ui.gps_sat0_elev_value,  self.ui.gps_sat1_elev_value,
                                        self.ui.gps_sat2_elev_value,  self.ui.gps_sat3_elev_value],
                                       [self.ui.gps_sat4_elev_value,  self.ui.gps_sat5_elev_value,
                                        self.ui.gps_sat6_elev_value,  self.ui.gps_sat7_elev_value],
                                       [self.ui.gps_sat8_elev_value,  self.ui.gps_sat9_elev_value,
                                        self.ui.gps_sat10_elev_value, self.ui.gps_sat11_elev_value]]
        
        self.ui.gps_azim_sat_values = [[self.ui.gps_sat0_azim_value,  self.ui.gps_sat1_azim_value,
                                        self.ui.gps_sat2_azim_value,  self.ui.gps_sat3_azim_value],
                                       [self.ui.gps_sat4_azim_value,  self.ui.gps_sat5_azim_value,
                                        self.ui.gps_sat6_azim_value,  self.ui.gps_sat7_azim_value],
                                       [self.ui.gps_sat8_azim_value,  self.ui.gps_sat9_azim_value,
                                        self.ui.gps_sat10_azim_value, self.ui.gps_sat11_azim_value]]
        
        self.ui.gps_snr_sat_values  = [[self.ui.gps_sat0_snr_value,  self.ui.gps_sat1_snr_value,
                                        self.ui.gps_sat2_snr_value,  self.ui.gps_sat3_snr_value],
                                       [self.ui.gps_sat4_snr_value,  self.ui.gps_sat5_snr_value,
                                        self.ui.gps_sat6_snr_value,  self.ui.gps_sat7_snr_value],
                                       [self.ui.gps_sat8_snr_value,  self.ui.gps_sat9_snr_value,
                                        self.ui.gps_sat10_snr_value, self.ui.gps_sat11_snr_value]]


    def setup_button_widgets(self):
        if bool(self.config['blue_led_button']['pressed_enable']):
            self.ui.blue_led_button.pressed.connect(lambda:self.publish_pin(publisher=self.blue_led_button_pub,
                                                                            pin=int(self.config['blue_led_button']['pin']),
                                                                            state=True))
        if bool(self.config['blue_led_button']['released_enable']):
            self.ui.blue_led_button.released.connect(lambda:self.publish_pin(publisher=self.blue_led_button_pub,
                                                                             pin=int(self.config['blue_led_button']['pin']),
                                                                             state=False))
            
        if bool(self.config['red_led_button']['pressed_enable']):
            self.ui.red_led_button.pressed.connect(lambda:self.publish_pin(publisher=self.red_led_button_pub,
                                                                           pin=int(self.config['red_led_button']['pin']),
                                                                           state=True))
        if bool(self.config['red_led_button']['released_enable']):
            self.ui.red_led_button.released.connect(lambda:self.publish_pin(publisher=self.red_led_button_pub,
                                                                            pin=int(self.config['red_led_button']['pin']),
                                                                            state=False))
            
        if bool(self.config['yellow_led_button']['pressed_enable']):
            self.ui.yellow_led_button.pressed.connect(lambda:self.publish_pin(publisher=self.yellow_led_button_pub,
                                                                              pin=int(self.config['yellow_led_button']['pin']),
                                                                              state=True))
        if bool(self.config['yellow_led_button']['released_enable']):
            self.ui.yellow_led_button.released.connect(lambda:self.publish_pin(publisher=self.yellow_led_button_pub,
                                                                               pin=int(self.config['yellow_led_button']['pin']),
                                                                               state=False))
            
        if bool(self.config['green_led_button']['pressed_enable']):
            self.ui.green_led_button.pressed.connect(lambda:self.publish_pin(publisher=self.green_led_button_pub,
                                                                             pin=int(self.config['green_led_button']['pin']),
                                                                             state=True))
        if bool(self.config['green_led_button']['released_enable']):
            self.ui.green_led_button.released.connect(lambda:self.publish_pin(publisher=self.green_led_button_pub,
                                                                              pin=int(self.config['green_led_button']['pin']),
                                                                              state=False))


    def setup_misc_widgets(self):
        self.ui.fan_speed_dial.sliderReleased.connect(self.fan_control_dial_callback)
        self.ui.ext_button_label.setPixmap(QtGui.QPixmap(ament_index_python.get_package_share_directory('road_quality_monitor') + '/red_led_off.png'))


    def setup_gauge_widgets(self):
        self.ui.gps_speed_gauge.units = "Km/h"
        self.ui.gps_speed_gauge.minValue = 0
        self.ui.gps_speed_gauge.maxValue = 120
        self.ui.gps_speed_gauge.scalaCount = 10
        self.ui.gps_speed_gauge.setEnableBarGraph(False)
        self.ui.gps_speed_gauge.setMouseTracking(False)
        self.ui.gps_speed_gauge.setGaugeTheme(2)
        self.ui.gps_speed_gauge.set_scale_polygon_colors([[.1,  QtCore.Qt.red],
                                                          [.45, QtCore.Qt.yellow],
                                                          [.75, QtCore.Qt.green]])
        self.ui.gps_speed_gauge.setScaleStartAngle(160)
        self.ui.gps_speed_gauge.setTotalScaleAngleSize(220)  
        self.ui.gps_speed_gauge.updateValue(0.0)


    def setup_plot_widgets(self):
        self.ui.acc_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.acc_x_plot = self.ui.acc_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.acc_y_plot = self.ui.acc_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.acc_z_plot = self.ui.acc_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.acc_plot.setBackground((34, 39, 40))
        self.ui.acc_plot.setLabel('left', 'm/s^2')
        self.ui.acc_plot.setLabel('bottom', 'sec')
        self.ui.acc_plot.showGrid(x=True, y=True, alpha=1.0)
        # self.ui.acc_plot.setRange(xRange=[-self.map_timeplots_show_last_secs, 0.0])

        self.ui.gyro_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.gyro_x_plot = self.ui.gyro_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.gyro_y_plot = self.ui.gyro_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.gyro_z_plot = self.ui.gyro_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.gyro_plot.setBackground((34, 39, 40))
        self.ui.gyro_plot.setLabel('left', 'rad/s')
        self.ui.gyro_plot.setLabel('bottom', 'sec')
        self.ui.gyro_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.magn_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.magn_x_plot = self.ui.magn_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.magn_y_plot = self.ui.magn_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.magn_z_plot = self.ui.magn_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.magn_plot.setBackground((34, 39, 40))
        self.ui.magn_plot.setLabel('left', 'uT')
        self.ui.magn_plot.setLabel('bottom', 'sec')
        self.ui.magn_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.acc_static_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.acc_static_calib_x_plot = self.ui.acc_static_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.acc_static_calib_y_plot = self.ui.acc_static_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.acc_static_calib_z_plot = self.ui.acc_static_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.acc_static_calib_plot.setBackground((34, 39, 40))
        self.ui.acc_static_calib_plot.setLabel('left', 'm/s^2')
        self.ui.acc_static_calib_plot.setLabel('bottom', 'sec')
        self.ui.acc_static_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.gyro_static_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.gyro_static_calib_x_plot = self.ui.gyro_static_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.gyro_static_calib_y_plot = self.ui.gyro_static_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.gyro_static_calib_z_plot = self.ui.gyro_static_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.gyro_static_calib_plot.setBackground((34, 39, 40))
        self.ui.gyro_static_calib_plot.setLabel('left', 'rad/s')
        self.ui.gyro_static_calib_plot.setLabel('bottom', 'sec')
        self.ui.gyro_static_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.magn_static_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.magn_static_calib_x_plot = self.ui.magn_static_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.magn_static_calib_y_plot = self.ui.magn_static_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.magn_static_calib_z_plot = self.ui.magn_static_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.magn_static_calib_plot.setBackground((34, 39, 40))
        self.ui.magn_static_calib_plot.setLabel('left', 'uT')
        self.ui.magn_static_calib_plot.setLabel('bottom', 'sec')
        self.ui.magn_static_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.acc_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.acc_calib_x_plot = self.ui.acc_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.acc_calib_y_plot = self.ui.acc_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.acc_calib_z_plot = self.ui.acc_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.acc_calib_plot.setBackground((34, 39, 40))
        self.ui.acc_calib_plot.setLabel('left', 'm/s^2')
        self.ui.acc_calib_plot.setLabel('bottom', 'sec')
        self.ui.acc_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.gyro_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.gyro_calib_x_plot = self.ui.gyro_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.gyro_calib_y_plot = self.ui.gyro_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.gyro_calib_z_plot = self.ui.gyro_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.gyro_calib_plot.setBackground((34, 39, 40))
        self.ui.gyro_calib_plot.setLabel('left', 'rad/s')
        self.ui.gyro_calib_plot.setLabel('bottom', 'sec')
        self.ui.gyro_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.ui.magn_calib_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.magn_calib_x_plot = self.ui.magn_calib_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.magn_calib_y_plot = self.ui.magn_calib_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.magn_calib_z_plot = self.ui.magn_calib_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.magn_calib_plot.setBackground((34, 39, 40))
        self.ui.magn_calib_plot.setLabel('left', 'uT')
        self.ui.magn_calib_plot.setLabel('bottom', 'sec')
        self.ui.magn_calib_plot.showGrid(x=True, y=True, alpha=1.0)

        self.update_plot_timer = self.create_timer(self.config['plot_update_period_ms'] / 1000, self.update_plots_callback)


    def gprmc_callback(self, msg: Gprmc):
        self.gprmc_parser.parse_msg(msg)

        self.ui.gprmc_hz_value.setText(f'{self.gprmc_parser.rate.value:.1f} Hz')

        self.ui.gps_datetime_value.setText(self.gprmc_parser.datetime)

        self.ui.gps_long_value.setText(f'{self.gprmc_parser.longitude_deg:.8f}° {self.gprmc_parser.longitude_dir}')
        self.ui.gps_lat_value.setText(f'{self.gprmc_parser.latitude_deg:.8f}° {self.gprmc_parser.latitude_dir}')

        self.ui.gps_pos_valid_value.setText('True' if self.gprmc_parser.pos_valid else 'False')
        if self.gprmc_parser.pos_valid:
            self.ui.gps_pos_valid_value.setStyleSheet("background-color: lightgreen")
        else:
            self.ui.gps_pos_valid_value.setStyleSheet("background-color: red")

        self.ui.gps_ground_speed_value.setText(f'{self.gprmc_parser.ground_speed:.2f} km/h')
        self.ui.gps_speed_gauge.updateValue(self.gprmc_parser.ground_speed)

        self.ui.gps_track_value.setText(f'{self.gprmc_parser.track_deg:.2f}°')
        self.ui.gps_compass.setAngle(self.gprmc_parser.track_deg)

        self.ui.gps_magn_var_angle_value.setText(f'{self.gprmc_parser.mag_var_deg}° {self.gprmc_parser.mag_var_dir}')


    def gpgga_callback(self, msg: Gpgga):
        self.gpgga_parser.parse_msg(msg)

        self.ui.gpgga_hz_value.setText(f'{self.gpgga_parser.rate.value:.1f} Hz')

        self.ui.gps_alt_value.setText(f'{self.gpgga_parser.altitude:.2f} {self.gpgga_parser.altitude_unit.lower()}')

        self.ui.gps_und_value.setText(f'{self.gpgga_parser.undulation:.2f} {self.gpgga_parser.undulation_unit.lower()}')

        self.ui.gps_pos_fixtype_value.setText(f'{self.gpgga_parser.pos_fix_type}')

        self.ui.gps_sats_used_value.setText(f'{self.gpgga_parser.n_satellites_used}')

    
    def gpgsa_callback(self, msg: Gpgsa):
        self.gpgsa_parser.parse_msg(msg)

        self.ui.gpgsa_hz_value.setText(f'{self.gpgsa_parser.rate.value:.1f} Hz')

        self.ui.gps_pdop_value.setText(f'{self.gpgsa_parser.pdop:.3f}')
        self.ui.gps_hdop_value.setText(f'{self.gpgsa_parser.hdop:.3f}')
        self.ui.gps_vdop_value.setText(f'{self.gpgsa_parser.vdop:.3f}')

        self.ui.gps_mode_value.setText(self.gpgsa_parser.mode)

        self.ui.gps_fixtype_value.setText(self.gpgsa_parser.fix_type)

        for i in range(12):
            self.ui.gps_id_sat_values[i].setText(f'{self.gpgsa_parser.sats_used_ids[i]}')

        
    def gpgsv_callback(self, msg: Gpgsv):
        self.gpgsv_parser.parse_msg(msg)

        self.ui.gpgsv_hz_value.setText(f'{self.gpgsv_parser.rate.value:.1f} Hz')

        self.ui.gps_sats_view_value.setText(f'{self.gpgsv_parser.n_satellites_view}')

        i = self.gpgsv_parser.current_sentence - 1

        if(i < 3):
            for j in range(4):
                self.ui.gps_prn_sat_values[i][j].setText(f'{self.gpgsv_parser.sat_view_prn[j]}')
                self.ui.gps_elev_sat_values[i][j].setText(f'{self.gpgsv_parser.sat_view_elevation_deg[j]}°')
                self.ui.gps_azim_sat_values[i][j].setText(f'{self.gpgsv_parser.sat_view_azimuth_deg[j]}°')
                self.ui.gps_snr_sat_values[i][j].setText(f'{self.gpgsv_parser.sat_view_snr[j]}dB')

    
    def pcd11_callback(self, msg: Pcd11):
        self.pcd11_parser.parse_msg(msg)

        self.ui.pcd11_hz_value.setText(f'{self.pcd11_parser.rate.value:.1f} Hz')

        if  (self.pcd11_parser.antenna_status == msg.INTERNAL_ANTENNA_USED):
            self.ui.gps_antenna_used_value.setText('Internal Antenna')
            self.ui.gps_antenna_used_value.setStyleSheet('')
        elif(self.pcd11_parser.antenna_status == msg.EXTERNAL_ANTENNA_USED):
            self.ui.gps_antenna_used_value.setText('External Antenna')
            self.ui.gps_antenna_used_value.setStyleSheet('background-color: lightgreen')
        elif(self.pcd11_parser.antenna_status == msg.EXTERNAL_ANTENNA_SHORTED):
            self.ui.gps_antenna_used_value.setText('External Antenna SHORTED!')
            self.ui.gps_antenna_used_value.setStyleSheet('background-color: red')
        else:
            self.ui.gps_antenna_used_value.setText('unknown')


    def imu_callback(self, msg: Imu):
        self.imu_counter = 0

        self.imu_parser.parse_msg(msg)

        self.ui.imu_raw_hz_value.setText(f'{self.imu_parser.rate.value:.1f} Hz')

        self.ui.acc_x_value.setText(f'{self.imu_parser.acc["x"]:.2f} m/s^2')
        self.ui.acc_y_value.setText(f'{self.imu_parser.acc["y"]:.2f} m/s^2')
        self.ui.acc_z_value.setText(f'{self.imu_parser.acc["z"]:.2f} m/s^2')

        self.ui.gyro_x_value.setText(f'{self.imu_parser.gyro["x"]:.2f} rad/s')
        self.ui.gyro_y_value.setText(f'{self.imu_parser.gyro["y"]:.2f} rad/s')
        self.ui.gyro_z_value.setText(f'{self.imu_parser.gyro["z"]:.2f} rad/s')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.imu_parser.acc_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_parser.acc_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_parser.acc_history["z"], self.config['imu_autoclear_ms'])

            autoclear_checker(self.imu_parser.gyro_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_parser.gyro_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_parser.gyro_history["z"], self.config['imu_autoclear_ms'])


    def imu_static_calib_callback(self, msg: Imu):
        self.imu_static_calib_counter = 0

        self.imu_static_calib_parser.parse_msg(msg)

        self.ui.imu_static_calib_hz_value.setText(f'{self.imu_static_calib_parser.rate.value:.1f} Hz')

        self.ui.acc_static_calib_x_value.setText(f'{self.imu_static_calib_parser.acc["x"]:.2f} m/s^2')
        self.ui.acc_static_calib_y_value.setText(f'{self.imu_static_calib_parser.acc["y"]:.2f} m/s^2')
        self.ui.acc_static_calib_z_value.setText(f'{self.imu_static_calib_parser.acc["z"]:.2f} m/s^2')

        self.ui.gyro_static_calib_x_value.setText(f'{self.imu_static_calib_parser.gyro["x"]:.2f} rad/s')
        self.ui.gyro_static_calib_y_value.setText(f'{self.imu_static_calib_parser.gyro["y"]:.2f} rad/s')
        self.ui.gyro_static_calib_z_value.setText(f'{self.imu_static_calib_parser.gyro["z"]:.2f} rad/s')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.imu_static_calib_parser.acc_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_static_calib_parser.acc_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_static_calib_parser.acc_history["z"], self.config['imu_autoclear_ms'])

            autoclear_checker(self.imu_static_calib_parser.gyro_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_static_calib_parser.gyro_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_static_calib_parser.gyro_history["z"], self.config['imu_autoclear_ms'])


    def imu_calib_callback(self, msg: Imu):
        self.imu_calib_counter = 0

        self.imu_calib_parser.parse_msg(msg)

        self.ui.imu_calib_hz_value.setText(f'{self.imu_calib_parser.rate.value:.1f} Hz')

        self.ui.acc_calib_x_value.setText(f'{self.imu_calib_parser.acc["x"]:.2f} m/s^2')
        self.ui.acc_calib_y_value.setText(f'{self.imu_calib_parser.acc["y"]:.2f} m/s^2')
        self.ui.acc_calib_z_value.setText(f'{self.imu_calib_parser.acc["z"]:.2f} m/s^2')

        self.ui.gyro_calib_x_value.setText(f'{self.imu_calib_parser.gyro["x"]:.2f} rad/s')
        self.ui.gyro_calib_y_value.setText(f'{self.imu_calib_parser.gyro["y"]:.2f} rad/s')
        self.ui.gyro_calib_z_value.setText(f'{self.imu_calib_parser.gyro["z"]:.2f} rad/s')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.imu_calib_parser.acc_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_calib_parser.acc_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_calib_parser.acc_history["z"], self.config['imu_autoclear_ms'])

            autoclear_checker(self.imu_calib_parser.gyro_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_calib_parser.gyro_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.imu_calib_parser.gyro_history["z"], self.config['imu_autoclear_ms'])


    def magn_callback(self, msg: Vector3Stamped):
        self.magn_counter = 0

        self.magn_parser.parse_msg(msg)

        self.ui.magn_raw_hz_value.setText(f'{self.magn_parser.rate.value:.1f} Hz')

        self.ui.magn_x_value.setText(f'{self.magn_parser.magn["x"]:.2f} uT')
        self.ui.magn_y_value.setText(f'{self.magn_parser.magn["y"]:.2f} uT')
        self.ui.magn_z_value.setText(f'{self.magn_parser.magn["z"]:.2f} uT')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.magn_parser.magn_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_parser.magn_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_parser.magn_history["z"], self.config['imu_autoclear_ms'])


    def magn_static_calib_callback(self, msg: Vector3Stamped):
        self.magn_static_calib_counter = 0

        self.magn_static_calib_parser.parse_msg(msg)

        self.ui.magn_static_calib_hz_value.setText(f'{self.magn_static_calib_parser.rate.value:.1f} Hz')

        self.ui.magn_static_calib_x_value.setText(f'{self.magn_static_calib_parser.magn["x"]:.2f} uT')
        self.ui.magn_static_calib_y_value.setText(f'{self.magn_static_calib_parser.magn["y"]:.2f} uT')
        self.ui.magn_static_calib_z_value.setText(f'{self.magn_static_calib_parser.magn["z"]:.2f} uT')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.magn_static_calib_parser.magn_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_static_calib_parser.magn_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_static_calib_parser.magn_history["z"], self.config['imu_autoclear_ms'])


    def magn_calib_callback(self, msg: Vector3Stamped):
        self.magn_calib_counter = 0

        self.magn_calib_parser.parse_msg(msg)

        self.ui.magn_calib_hz_value.setText(f'{self.magn_calib_parser.rate.value:.1f} Hz')

        self.ui.magn_calib_x_value.setText(f'{self.magn_calib_parser.magn["x"]:.2f} uT')
        self.ui.magn_calib_y_value.setText(f'{self.magn_calib_parser.magn["y"]:.2f} uT')
        self.ui.magn_calib_z_value.setText(f'{self.magn_calib_parser.magn["z"]:.2f} uT')

        if(bool(self.config['imu_autoclear'])):
            autoclear_checker(self.magn_calib_parser.magn_history["x"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_calib_parser.magn_history["y"], self.config['imu_autoclear_ms'])
            autoclear_checker(self.magn_calib_parser.magn_history["z"], self.config['imu_autoclear_ms'])


    def camera_callback(self, msg: CompressedImage):
        qimage = QtGui.QImage.fromData(msg.data)
        pixmap = QtGui.QPixmap.fromImage(qimage)

        current_camera_msg_time = get_stamp_time_sec(msg.header.stamp)
        fps = 1 / (current_camera_msg_time - self.last_camera_msg_time)
        self.last_camera_msg_time = current_camera_msg_time
        self.camera_fps_filter.insert(fps)

        self.ui.camera_resolution_value.setText(f'{qimage.width()}x{qimage.height()}')
        self.ui.camera_fps_value.setText(f'{self.camera_fps_filter.value:.1f}')

        size = self.ui.camera_view.size()
        self.ui.camera_view.setPixmap(pixmap.scaled(size, QtCore.Qt.KeepAspectRatio))


    def external_button_callback(self, msg: PinState):
        if msg.state:
            self.ui.ext_button_label.setPixmap(QtGui.QPixmap(ament_index_python.get_package_share_directory('road_quality_monitor') + '/red_led_on.png'))
        else:
            self.ui.ext_button_label.setPixmap(QtGui.QPixmap(ament_index_python.get_package_share_directory('road_quality_monitor') + '/red_led_off.png'))


    def check_imu_calib(self):
        self.imu_counter += 1
        self.imu_static_calib_counter += 1
        self.imu_calib_counter += 1

        self.magn_counter += 1
        self.magn_static_calib_counter += 1
        self.magn_calib_counter += 1

        if self.imu_counter < 5:
            self.imu_is_published = True
            self.ui.imu_state_value.setText('Publishing')
            self.ui.imu_state_value.setStyleSheet('color: lightgreen')
        else:
            self.imu_is_published = False
            self.ui.imu_state_value.setText('Unavailable')
            self.ui.imu_state_value.setStyleSheet('color: rgb(255,255,255)')

        if self.imu_static_calib_counter < 5:
            self.imu_static_calib_is_published = True
            self.ui.imu_static_calib_state_value.setText('Publishing')
            self.ui.imu_static_calib_state_value.setStyleSheet('color: lightgreen')
        else:
            self.imu_static_calib_is_published = False
            self.ui.imu_static_calib_state_value.setText('Unavailable')
            self.ui.imu_static_calib_state_value.setStyleSheet('color: rgb(255,255,255)')

        if self.imu_calib_counter < 5:
            self.imu_calib_is_published = True
            self.ui.imu_calib_state_value.setText('Publishing')
            self.ui.imu_calib_state_value.setStyleSheet('color: lightgreen')
        else:
            self.imu_calib_is_published = False
            self.ui.imu_calib_state_value.setText('Unavailable')
            self.ui.imu_calib_state_value.setStyleSheet('color: rgb(255,255,255)')

        if self.magn_counter < 5:
            self.magn_is_published = True
            self.ui.magn_state_value.setText('Publishing')
            self.ui.magn_state_value.setStyleSheet('color: lightgreen')
        else:
            self.magn_is_published = False
            self.ui.magn_state_value.setText('Unavailable')
            self.ui.magn_state_value.setStyleSheet('color: rgb(255,255,255)')

        if self.magn_static_calib_counter < 5:
            self.magn_static_calib_is_published = True
            self.ui.magn_static_calib_state_value.setText('Publishing')
            self.ui.magn_static_calib_state_value.setStyleSheet('color: lightgreen')
        else:
            self.magn_static_calib_is_published = False
            self.ui.magn_static_calib_state_value.setText('Unavailable')
            self.ui.magn_static_calib_state_value.setStyleSheet('color: rgb(255,255,255)')

        if self.magn_calib_counter < 5:
            self.magn_calib_is_published = True
            self.ui.magn_calib_state_value.setText('Publishing')
            self.ui.magn_calib_state_value.setStyleSheet('color: lightgreen')
        else:
            self.magn_calib_is_published = False
            self.ui.magn_calib_state_value.setText('Unavailable')
            self.ui.magn_calib_state_value.setStyleSheet('color: rgb(255,255,255)')


    def check_active_nodes(self):
        self.active_nodes = dict.fromkeys(self.active_nodes, False)
        for node in self.get_node_names():
            if node == 'nucleus':
                self.active_nodes['nucleus'] = True
            
            if node == 'gpio_handler':
                self.active_nodes['gpio'] = True

            if node == 'icm20948_reader':
                self.active_nodes['imu'] = True

            if node == 'mtk3339_cpp_reader':
                self.active_nodes['gps'] = True

            if 'imu_static_calib' in node:
                self.active_nodes['static_calib'] = True

            if 'imu_dyn_calib' in node:
                self.active_nodes['dyn_calib'] = True

            if node == 'rosbag2_recorder':
                self.active_nodes['rosbag_rec'] = True

            if node == 'camera_record':
                self.active_nodes['camera'] = True

        for key in self.active_nodes.keys():
            if self.active_nodes[key]:
                self.active_nodes_ui_element[key].setStyleSheet('color: lightgreen')
            else:
                self.active_nodes_ui_element[key].setStyleSheet('color: rgb(255,255,255)')


    def update_plots_callback(self):
        if self.imu_is_published:
            update_timeplot(self.acc_x_plot, self.imu_parser.acc_history["x"])
            update_timeplot(self.acc_y_plot, self.imu_parser.acc_history["y"])
            update_timeplot(self.acc_z_plot, self.imu_parser.acc_history["z"])

            update_timeplot(self.gyro_x_plot, self.imu_parser.gyro_history["x"])
            update_timeplot(self.gyro_y_plot, self.imu_parser.gyro_history["y"])
            update_timeplot(self.gyro_z_plot, self.imu_parser.gyro_history["z"])

        if self.imu_static_calib_is_published:
            update_timeplot(self.acc_static_calib_x_plot, self.imu_static_calib_parser.acc_history["x"])
            update_timeplot(self.acc_static_calib_y_plot, self.imu_static_calib_parser.acc_history["y"])
            update_timeplot(self.acc_static_calib_z_plot, self.imu_static_calib_parser.acc_history["z"])

            update_timeplot(self.gyro_static_calib_x_plot, self.imu_static_calib_parser.gyro_history["x"])
            update_timeplot(self.gyro_static_calib_y_plot, self.imu_static_calib_parser.gyro_history["y"])
            update_timeplot(self.gyro_static_calib_z_plot, self.imu_static_calib_parser.gyro_history["z"])

        if self.imu_calib_is_published:
            update_timeplot(self.acc_calib_x_plot, self.imu_calib_parser.acc_history["x"])
            update_timeplot(self.acc_calib_y_plot, self.imu_calib_parser.acc_history["y"])
            update_timeplot(self.acc_calib_z_plot, self.imu_calib_parser.acc_history["z"])

            update_timeplot(self.gyro_calib_x_plot, self.imu_calib_parser.gyro_history["x"])
            update_timeplot(self.gyro_calib_y_plot, self.imu_calib_parser.gyro_history["y"])
            update_timeplot(self.gyro_calib_z_plot, self.imu_calib_parser.gyro_history["z"])

        if self.magn_is_published:
            update_timeplot(self.magn_x_plot, self.magn_parser.magn_history["x"])
            update_timeplot(self.magn_y_plot, self.magn_parser.magn_history["y"])
            update_timeplot(self.magn_z_plot, self.magn_parser.magn_history["z"])

        if self.magn_static_calib_is_published:
            update_timeplot(self.magn_static_calib_x_plot, self.magn_static_calib_parser.magn_history["x"])
            update_timeplot(self.magn_static_calib_y_plot, self.magn_static_calib_parser.magn_history["y"])
            update_timeplot(self.magn_static_calib_z_plot, self.magn_static_calib_parser.magn_history["z"])

        if self.magn_calib_is_published:
            update_timeplot(self.magn_calib_x_plot, self.magn_calib_parser.magn_history["x"])
            update_timeplot(self.magn_calib_y_plot, self.magn_calib_parser.magn_history["y"])
            update_timeplot(self.magn_calib_z_plot, self.magn_calib_parser.magn_history["z"])

    
    def publish_pin(self, publisher: Publisher, pin: int, state: bool):
        msg = PinState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pin = pin
        msg.state = state

        publisher.publish(msg)


    def fan_control_dial_callback(self):
        self.ui.fan_speed_dial_value.setText(f'{self.ui.fan_speed_dial.value():d}%')
        self.publish_pin_pwm(publisher=self.fan_control_dial_pub,
                             pin=int(self.config['fan_control_pin']),
                             freq=25000,
                             duty=(float(self.ui.fan_speed_dial.value()) / 100))


    def publish_pin_pwm(self, publisher: Publisher, pin: int, freq: int, duty: float):
        msg = PinPwmState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pin = pin
        msg.frequency = int(freq)
        msg.duty = int(1000000 * duty)

        publisher.publish(msg)
