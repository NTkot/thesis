import rclpy
from imu_static_calib import ImuStaticCalib, ImuStaticCalibReverseParking, ImuStaticCalibUserValues
from imu_dyn_calib import ImuDynCalibMagn, ImuDynCalibBrake, ImuDynCalibUserValues


def spin_node_until_exception(node):
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("imu_static_calib").info('Quitting...')

    node.destroy_node()
    rclpy.shutdown()


def static_calib_main(args=None):
    rclpy.init(args=args)
    imu_static_calib_node = ImuStaticCalib()
    spin_node_until_exception(imu_static_calib_node)


def static_calib_rp_main(args=None):
    rclpy.init(args=args)
    imu_static_calib_rp_node = ImuStaticCalibReverseParking()
    spin_node_until_exception(imu_static_calib_rp_node)


def static_calib_uv_main(args=None):
    rclpy.init(args=args)
    imu_static_calib_uv_node = ImuStaticCalibUserValues()
    spin_node_until_exception(imu_static_calib_uv_node)


def dyn_calib_magn_main(args=None):
    rclpy.init(args=args)
    imu_dyn_calib_magn_node = ImuDynCalibMagn()
    spin_node_until_exception(imu_dyn_calib_magn_node)


def dyn_calib_brake_main(args=None):
    rclpy.init(args=args)
    imu_dyn_calib_brake_node = ImuDynCalibBrake()
    spin_node_until_exception(imu_dyn_calib_brake_node)


def dyn_calib_uv_main(args=None):
    rclpy.init(args=args)
    imu_dyn_calib_uv_node = ImuDynCalibUserValues()
    spin_node_until_exception(imu_dyn_calib_uv_node)
