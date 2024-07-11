#ifndef ICM20948_READER_HPP
#define ICM20948_READER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "road_quality_msgs/msg/imu.hpp"
#include "road_quality_msgs/srv/get_imu_calib.hpp"

#include "icm20948_i2c.hpp"
#include "icm20948_defs.hpp"


class ICM20948Reader : public rclcpp::Node 
{
    private:
        ICM20948_i2c _sensor;

        struct {
            float pub_imu_hz;
            float pub_magn_hz;
            bool pub_raw;
            bool pub_static_calib;
            bool pub_calib;
        } _ros_settings;

        rclcpp::Publisher<road_quality_msgs::msg::Imu>::SharedPtr _raw_imu_publisher, 
                                                                  _static_calib_imu_publisher,
                                                                  _calib_imu_publisher;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _raw_magn_publisher, 
                                                                         _static_calib_magn_publisher,
                                                                         _calib_magn_publisher;

        road_quality_msgs::msg::Imu _raw_imu_msg, 
                                    _static_calib_imu_msg, 
                                    _calib_imu_msg;

        geometry_msgs::msg::Vector3Stamped _raw_magn_msg,
                                           _static_calib_magn_msg,
                                           _calib_magn_msg;

        rclcpp::TimerBase::SharedPtr _pub_imu_timer, _pub_magn_timer, _calib_timer;

        bool _static_calibrated, _calibrated;
        rclcpp::Client<road_quality_msgs::srv::GetImuCalib>::SharedPtr _calib_client;
        geometry_msgs::msg::TransformStamped _last_static_calib, _last_calib;
        tf2::Matrix3x3 _static_calib_rot_mat, _calib_rot_mat;

        bool _imu_init();
        void _parse_ros_settings(std::string config_filepath);
        void _ros_init();

        void _imu_pub_callback();
        void _magn_pub_callback();
        void _magn_reorient(const float *in, float *out);

        void _calib_request();
        void _calib_response_callback(rclcpp::Client<road_quality_msgs::srv::GetImuCalib>::SharedFuture future);

    public:
        ICM20948Reader(unsigned i2c_bus, unsigned i2c_address, std::string config_filepath);
};

#endif