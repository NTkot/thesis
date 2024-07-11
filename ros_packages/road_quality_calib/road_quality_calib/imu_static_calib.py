import math
import numpy as np
import os
import yaml

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster

from road_quality_msgs.msg import Imu, PinState
from geometry_msgs.msg import TransformStamped

from calib_utils import MovingAverage, static_calib, rotate_180deg_x


class ImuStaticCalib(Node):
    def __init__(self):
        super().__init__('imu_static_calib')

        self.param_init()

        self.var_init()

        self.ros_init()
        
        self.get_logger().info(f'Static IMU calibration initialized with window size = {self.window_size}')


    def param_init(self):
        self.declare_parameter('window_size', 100)


    def var_init(self):
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        self.accel_filter = {'x' : MovingAverage(self.window_size),
                             'y' : MovingAverage(self.window_size),
                             'z' : MovingAverage(self.window_size)}
        
        self.msgs_received = 0

        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = 'imu'
        self.tf_msg.child_frame_id = 'imu_static_calib'
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0


    def ros_init(self):
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_sub = self.create_subscription(Imu, '/imu/raw', 
                                                self.imu_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)

        
    def imu_callback(self, msg : Imu):
        if(self.msgs_received < self.window_size):
            self.msgs_received += 1
            self.accel_filter['x'].insert(msg.linear_acceleration.x)
            self.accel_filter['y'].insert(msg.linear_acceleration.y)
            self.accel_filter['z'].insert(msg.linear_acceleration.z)
        elif(self.msgs_received == self.window_size):
            self.msgs_received += 1
            self.perform_calib(self.accel_filter['x'].value(),
                               self.accel_filter['y'].value(),
                               self.accel_filter['z'].value())
        else:
            pass


    def perform_calib(self, a_imu_x, a_imu_y, a_imu_z):
        self.tf_msg.header.stamp = self.get_clock().now().to_msg()

        x_angle, y_angle = static_calib.accel_to_xy_angles(a_imu_x, a_imu_y, a_imu_z)
        rot = static_calib.xy_angles_to_rot(x_angle, y_angle)

        a_z_transformed = rot.apply(np.array([a_imu_x, a_imu_y, a_imu_z]))[2]
        if(a_z_transformed < 0):
            rot = rotate_180deg_x(rot)
            self.get_logger().info('Rotating around X by 180° to make a_z_static positive')

        quat = rot.as_quat()
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.get_logger().info('Static IMU calibration done, found:\n'
                              f'x_angle = {x_angle * 180 / math.pi:.3f}°' +
                             (f' (final = {math.atan2(math.sin(x_angle + math.pi), math.cos(x_angle + math.pi)) * 180 / math.pi:.3f}°)\n' if a_z_transformed < 0 else '\n') +
                              f'y_angle = {y_angle * 180 / math.pi:.3f}°')

        rot_mat = rot.as_matrix()
        self.get_logger().info('Rotation matrix (static_calib <- imu):\n'
                              f'[{rot_mat[0][0]:.4f} {rot_mat[0][1]:.4f} {rot_mat[0][2]:.4f}\n'
                              f' {rot_mat[1][0]:.4f} {rot_mat[1][1]:.4f} {rot_mat[1][2]:.4f}\n'
                              f' {rot_mat[2][0]:.4f} {rot_mat[2][1]:.4f} {rot_mat[2][2]:.4f}]')
        
        self.get_logger().info('Acceleration values used:\n'
                              f'a_x: {a_imu_x:.3f} m/s^2\n'
                              f'a_y: {a_imu_y:.3f} m/s^2\n'
                              f'a_z: {a_imu_z:.3f} m/s^2')
        
        self.tf_broadcaster.sendTransform(self.tf_msg)

        raise SystemExit




class ImuStaticCalibReverseParking(ImuStaticCalib):
    def __init__(self):
        super(ImuStaticCalib, self).__init__('imu_static_calib')

        self.param_init()

        self.var_init()

        self.ros_init()

        self.get_logger().info(f'Static IMU calibration initialized with window size = {self.window_size}'
                                ' and reverse parking method')


    def param_init(self):
        super().param_init()

        self.declare_parameter('phase_button_pin', 21)
        self.declare_parameter('phase_led_pin',    5)


    def var_init(self):
        super().var_init()

        self.phase_button_pin = self.get_parameter('phase_button_pin').get_parameter_value().integer_value

        self.reverse_parking_phase = 0
        self.reverse_parking_waiting_for_button = False

        self.led_msg = PinState()
        self.led_msg.pin = self.get_parameter('phase_led_pin').get_parameter_value().integer_value


    def ros_init(self):
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_sub = self.create_subscription(Imu, '/imu/raw', 
                                                self.imu_rp_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)

        self.reverse_parking_button_sub = self.create_subscription(PinState, f'/gpio/inputs/pin{self.phase_button_pin:d}',
                                                                   self.reverse_parking_button_callback,
                                                                   QoSPresetProfiles.SERVICES_DEFAULT.value)
                                                                   
        self.reverse_parking_led_pub = self.create_publisher(PinState, f'/gpio/outputs/pin{self.led_msg.pin:d}',
                                                             QoSPresetProfiles.SERVICES_DEFAULT.value)
        

    def imu_rp_callback(self, msg: Imu):
        if(self.msgs_received < self.window_size):
            self.msgs_received += 1
            self.accel_filter['x'].insert(msg.linear_acceleration.x)
            self.accel_filter['y'].insert(msg.linear_acceleration.y)
            self.accel_filter['z'].insert(msg.linear_acceleration.z)
        elif(self.msgs_received == self.window_size):
            self.msgs_received += 1
            if(self.reverse_parking_phase == 0):
                self.a_imu_x_phase_0 = self.accel_filter['x'].value()
                self.a_imu_y_phase_0 = self.accel_filter['y'].value()
                self.a_imu_z_phase_0 = self.accel_filter['z'].value()

                self.accel_filter['x'].clear()
                self.accel_filter['y'].clear()
                self.accel_filter['z'].clear()

                self.reverse_parking_waiting_for_button = True

                self.led_msg.header.stamp = self.get_clock().now().to_msg()
                self.led_msg.state = True
                self.reverse_parking_led_pub.publish(self.led_msg)

                self.get_logger().info('First phase of static calibration with reverse parking is FINISHED')
            else:
                self.a_imu_x_phase_1 = self.accel_filter['x'].value()
                self.a_imu_y_phase_1 = self.accel_filter['y'].value()
                self.a_imu_z_phase_1 = self.accel_filter['z'].value()

                a_imu_x = (self.a_imu_x_phase_0 + self.a_imu_x_phase_1) / 2
                a_imu_y = (self.a_imu_y_phase_0 + self.a_imu_y_phase_1) / 2
                a_imu_z = (self.a_imu_z_phase_0 + self.a_imu_z_phase_1) / 2

                self.led_msg.header.stamp = self.get_clock().now().to_msg()
                self.led_msg.state = False
                self.reverse_parking_led_pub.publish(self.led_msg)

                super().perform_calib(a_imu_x, a_imu_y, a_imu_z)
        else:
            pass
        

    def reverse_parking_button_callback(self, msg : PinState):
        if self.reverse_parking_waiting_for_button:
            self.get_logger().info('Button for second phase of static calibration with reverse parking PRESSED')
            self.reverse_parking_waiting_for_button = False
            self.reverse_parking_phase = 1
            self.msgs_received = 0




class ImuStaticCalibUserValues(Node):
    def __init__(self):
        super().__init__('imu_static_calib')

        self.param_init()

        self.var_init()

        self.ros_init()

        self.get_logger().info('Static IMU calibration initialized with user values')


    def param_init(self):
        self.declare_parameter('x_angle', 0.0)
        self.declare_parameter('y_angle', 0.0)
        self.declare_parameter('calib_file', '')


    def var_init(self):
        self.x_angle_param = self.get_parameter('x_angle').get_parameter_value().double_value
        self.y_angle_param = self.get_parameter('y_angle').get_parameter_value().double_value
        self.calib_file = self.get_parameter('calib_file').get_parameter_value().string_value

        self.accel_filter = {'x' : MovingAverage(10),
                             'y' : MovingAverage(10),
                             'z' : MovingAverage(10)}
        
        self.msgs_received = 0

        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = 'imu'
        self.tf_msg.child_frame_id = 'imu_static_calib'
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0


    def ros_init(self):
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_sub = self.create_subscription(Imu, '/imu/raw', 
                                                self.imu_uv_callback, 
                                                QoSPresetProfiles.SENSOR_DATA.value)


    def imu_uv_callback(self, msg: Imu):
        if(self.msgs_received < 10):
            self.msgs_received += 1
            self.accel_filter['x'].insert(msg.linear_acceleration.x)
            self.accel_filter['y'].insert(msg.linear_acceleration.y)
            self.accel_filter['z'].insert(msg.linear_acceleration.z)
        elif(self.msgs_received == 10):
            self.msgs_received += 1
            self.publish_user_tf()
        else:
            pass


    def publish_user_tf(self):
        if self.calib_file and os.path.isfile(self.calib_file):
            with open(self.calib_file) as f:
                calib_file_yaml = yaml.load(f, Loader=yaml.FullLoader)
            x_angle = calib_file_yaml['x_angle']
            y_angle = calib_file_yaml['y_angle']
            self.get_logger().info(f'Using calib file: {self.calib_file}')
        else:
            x_angle = self.x_angle_param
            y_angle = self.y_angle_param
            self.get_logger().info(f'Using passed ROS parameters as angles')

        rot = static_calib.xy_angles_to_rot(x_angle, y_angle)

        a_imu_x = self.accel_filter['x'].value()
        a_imu_y = self.accel_filter['y'].value()
        a_imu_z = self.accel_filter['z'].value()

        a_z_transformed = rot.apply(np.array([a_imu_x, a_imu_y, a_imu_z]))[2]
        if(a_z_transformed < 0):
            rot = rotate_180deg_x(rot)
            self.get_logger().info('Rotating around X by 180° to make a_z_static positive')

        quat = rot.as_quat()
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.get_logger().info('Static IMU calibration done, found:\n'
                              f'x_angle = {x_angle * 180 / math.pi:.3f}°' +
                             (f' (final = {math.atan2(math.sin(x_angle + math.pi), math.cos(x_angle + math.pi)) * 180 / math.pi:.3f}°)\n' if a_z_transformed < 0 else '\n') +
                              f'y_angle = {y_angle * 180 / math.pi:.3f}°')

        rot_mat = rot.as_matrix()
        self.get_logger().info('Rotation matrix (static_calib <- imu):\n'
                              f'[{rot_mat[0][0]:.4f} {rot_mat[0][1]:.4f} {rot_mat[0][2]:.4f}\n'
                              f' {rot_mat[1][0]:.4f} {rot_mat[1][1]:.4f} {rot_mat[1][2]:.4f}\n'
                              f' {rot_mat[2][0]:.4f} {rot_mat[2][1]:.4f} {rot_mat[2][2]:.4f}]')
        
        self.tf_broadcaster.sendTransform(self.tf_msg)

        raise SystemExit
