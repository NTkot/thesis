#include "icm20948_reader.hpp"
#include "icm20948_utils.hpp"

#include <chrono>
#include <thread>

#include "yaml-cpp/yaml.h"


ICM20948Reader::ICM20948Reader(unsigned i2c_bus, unsigned i2c_address, std::string config_filepath) : Node("icm20948_reader"),
                                                                                                      _sensor(i2c_bus,
                                                                                                              i2c_address,
                                                                                                              icm20948::settings(YAML::LoadFile(config_filepath.c_str()))),
                                                                                                      _static_calibrated(false),
                                                                                                      _calibrated(false)

{
    if(!_imu_init())
    {
        rclcpp::shutdown(nullptr, "Could not initialize ICM20948 sensor");
    }

    _parse_ros_settings(config_filepath);

    _ros_init();

    RCLCPP_INFO(this->get_logger(), "IMU sensor/node initialized with following settings:\n"
                                    "Accelerometer:\n"
                                    "  Sample rate: %.1fHz\n"
                                    "  Scale: %s\n"
                                    "  DLPF:\n"
                                    "    Enable: %d\n"
                                    "    Cutoff: %s\n"
                                    "Gyroscope:\n"
                                    "  Sample rate: %.1fHz\n"
                                    "  Scale: %s\n"
                                    "  DLPF:\n"
                                    "    Enable: %d\n"
                                    "    Cutoff: %s\n"
                                    "Magnetometer:\n"
                                    "  Mode: %s\n"
                                    "ROS:\n"
                                    "  Publish IMU rate:  %.1fHz\n"
                                    "  Publish Magn rate: %.1fHz\n"
                                    "  Publish raw: %d\n"
                                    "  Publish static calib: %d\n"
                                    "  Publish calib: %d\n", 1125.0f / (1+_sensor.settings.accel.sample_rate_div),
                                                             icm20948::accel_scale_to_str(_sensor.settings.accel.scale).c_str(),
                                                             _sensor.settings.accel.dlpf_enable,
                                                             icm20948::accel_dlpf_config_to_str(_sensor.settings.accel.dlpf_config).c_str(),
                                                             1100.0f / (1+_sensor.settings.gyro.sample_rate_div),
                                                             icm20948::gyro_scale_to_str(_sensor.settings.gyro.scale).c_str(),
                                                             _sensor.settings.gyro.dlpf_enable,
                                                             icm20948::gyro_dlpf_config_to_str(_sensor.settings.gyro.dlpf_config).c_str(),
                                                             icm20948::magn_mode_to_str(_sensor.settings.magn.mode).c_str(),
                                                             _ros_settings.pub_imu_hz,
                                                             _ros_settings.pub_magn_hz,
                                                             _ros_settings.pub_raw,
                                                             _ros_settings.pub_static_calib,
                                                             _ros_settings.pub_calib);
}


bool ICM20948Reader::_imu_init()
{
    bool success = _sensor.init();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    return success;
}


void ICM20948Reader::_parse_ros_settings(std::string config_filepath)
{
    YAML::Node config_file = YAML::LoadFile(config_filepath.c_str());

    for(YAML::const_iterator it = config_file.begin(); it != config_file.end(); ++it)
    {
        if(it->first.as<std::string>() == "ros")
        {
            for(YAML::const_iterator ros_it = it->second.begin(); ros_it != it->second.end(); ++ros_it)
            {
                if(ros_it->first.as<std::string>() == "pub_imu_hz")
                {
                    _ros_settings.pub_imu_hz = ros_it->second.as<float>();
                }
                else if(ros_it->first.as<std::string>() == "pub_magn_hz")
                {
                    _ros_settings.pub_magn_hz = ros_it->second.as<float>();
                }
                else if(ros_it->first.as<std::string>() == "pub_raw")
                {
                    _ros_settings.pub_raw = (bool)ros_it->second.as<int>();
                }
                else if(ros_it->first.as<std::string>() == "pub_static_calib")
                {
                    _ros_settings.pub_static_calib = (bool)ros_it->second.as<int>();
                }
                else if(ros_it->first.as<std::string>() == "pub_calib")
                {
                    _ros_settings.pub_calib = (bool)ros_it->second.as<int>();
                }
            }
        }
    }
}


void ICM20948Reader::_ros_init()
{
    _raw_imu_msg.header.frame_id = "imu";
    _static_calib_imu_msg.header.frame_id = "imu_static_calib";
    _calib_imu_msg.header.frame_id = "imu_calib";

    _raw_magn_msg.header.frame_id = "imu";
    _static_calib_magn_msg.header.frame_id = "imu_static_calib";
    _calib_magn_msg.header.frame_id = "imu_calib";

    rclcpp::SensorDataQoS sensorQos;
    if(_ros_settings.pub_raw)
    {
        _raw_imu_publisher  = this->create_publisher<road_quality_msgs::msg::Imu>("/imu/raw", sensorQos);
        _raw_magn_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/magn/raw", sensorQos);
    }
    if(_ros_settings.pub_static_calib)
    {
        _static_calib_imu_publisher  = this->create_publisher<road_quality_msgs::msg::Imu>("/imu/static_calib", sensorQos);
        _static_calib_magn_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/magn/static_calib", sensorQos);
    }
    if(_ros_settings.pub_calib)
    {
        _calib_imu_publisher  = this->create_publisher<road_quality_msgs::msg::Imu>("/imu/calib", sensorQos);
        _calib_magn_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/magn/calib", sensorQos);
    }

    float imu_period_ms  = (1 / _ros_settings.pub_imu_hz)  * 1000;
    float magn_period_ms = (1 / _ros_settings.pub_magn_hz) * 1000;
    _pub_imu_timer  = this->create_wall_timer(std::chrono::milliseconds((int)imu_period_ms),  std::bind(&ICM20948Reader::_imu_pub_callback,  this));
    _pub_magn_timer = this->create_wall_timer(std::chrono::milliseconds((int)magn_period_ms), std::bind(&ICM20948Reader::_magn_pub_callback, this));

    if(_ros_settings.pub_static_calib || _ros_settings.pub_calib)
    {
        _calib_client = this->create_client<road_quality_msgs::srv::GetImuCalib>("get_imu_calib");
        _calib_timer  = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ICM20948Reader::_calib_request, this));
    }

}


void ICM20948Reader::_imu_pub_callback()
{
    _sensor.read_accel_gyro();
    _raw_imu_msg.header.stamp = rclcpp::Node::now();

    if(_ros_settings.pub_raw)
    {
        _raw_imu_msg.linear_acceleration.x = _sensor.accel[0];
        _raw_imu_msg.linear_acceleration.y = _sensor.accel[1];
        _raw_imu_msg.linear_acceleration.z = _sensor.accel[2];

        _raw_imu_msg.angular_velocity.x = _sensor.gyro[0];
        _raw_imu_msg.angular_velocity.y = _sensor.gyro[1];
        _raw_imu_msg.angular_velocity.z = _sensor.gyro[2];

        _raw_imu_publisher->publish(_raw_imu_msg);
    }
    if(_ros_settings.pub_static_calib && _static_calibrated)
    {
        _static_calib_imu_msg.header.stamp = _raw_imu_msg.header.stamp;

        _static_calib_imu_msg.linear_acceleration.x = _static_calib_rot_mat[0][0] * _sensor.accel[0] +
                                                      _static_calib_rot_mat[0][1] * _sensor.accel[1] +
                                                      _static_calib_rot_mat[0][2] * _sensor.accel[2];
        _static_calib_imu_msg.linear_acceleration.y = _static_calib_rot_mat[1][0] * _sensor.accel[0] +
                                                      _static_calib_rot_mat[1][1] * _sensor.accel[1] +
                                                      _static_calib_rot_mat[1][2] * _sensor.accel[2];
        _static_calib_imu_msg.linear_acceleration.z = _static_calib_rot_mat[2][0] * _sensor.accel[0] +
                                                      _static_calib_rot_mat[2][1] * _sensor.accel[1] +
                                                      _static_calib_rot_mat[2][2] * _sensor.accel[2];

        _static_calib_imu_msg.angular_velocity.x = _static_calib_rot_mat[0][0] * _sensor.gyro[0] +
                                                   _static_calib_rot_mat[0][1] * _sensor.gyro[1] +
                                                   _static_calib_rot_mat[0][2] * _sensor.gyro[2];
        _static_calib_imu_msg.angular_velocity.y = _static_calib_rot_mat[1][0] * _sensor.gyro[0] +
                                                   _static_calib_rot_mat[1][1] * _sensor.gyro[1] +
                                                   _static_calib_rot_mat[1][2] * _sensor.gyro[2];
        _static_calib_imu_msg.angular_velocity.z = _static_calib_rot_mat[2][0] * _sensor.gyro[0] +
                                                   _static_calib_rot_mat[2][1] * _sensor.gyro[1] +
                                                   _static_calib_rot_mat[2][2] * _sensor.gyro[2];

        _static_calib_imu_publisher->publish(_static_calib_imu_msg);
    }
    if(_ros_settings.pub_calib && _calibrated)
    {
        _calib_imu_msg.header.stamp = _raw_imu_msg.header.stamp;

        _calib_imu_msg.linear_acceleration.x = _calib_rot_mat[0][0] * _sensor.accel[0] +
                                               _calib_rot_mat[0][1] * _sensor.accel[1] +
                                               _calib_rot_mat[0][2] * _sensor.accel[2];
        _calib_imu_msg.linear_acceleration.y = _calib_rot_mat[1][0] * _sensor.accel[0] +
                                               _calib_rot_mat[1][1] * _sensor.accel[1] +
                                               _calib_rot_mat[1][2] * _sensor.accel[2];
        _calib_imu_msg.linear_acceleration.z = _calib_rot_mat[2][0] * _sensor.accel[0] +
                                               _calib_rot_mat[2][1] * _sensor.accel[1] +
                                               _calib_rot_mat[2][2] * _sensor.accel[2];

        _calib_imu_msg.angular_velocity.x = _calib_rot_mat[0][0] * _sensor.gyro[0] +
                                            _calib_rot_mat[0][1] * _sensor.gyro[1] +
                                            _calib_rot_mat[0][2] * _sensor.gyro[2];
        _calib_imu_msg.angular_velocity.y = _calib_rot_mat[1][0] * _sensor.gyro[0] +
                                            _calib_rot_mat[1][1] * _sensor.gyro[1] +
                                            _calib_rot_mat[1][2] * _sensor.gyro[2];
        _calib_imu_msg.angular_velocity.z = _calib_rot_mat[2][0] * _sensor.gyro[0] +
                                            _calib_rot_mat[2][1] * _sensor.gyro[1] +
                                            _calib_rot_mat[2][2] * _sensor.gyro[2];

        _calib_imu_publisher->publish(_calib_imu_msg);
    }
}

void ICM20948Reader::_magn_pub_callback()
{
    _sensor.read_magn();
    _raw_magn_msg.header.stamp = rclcpp::Node::now();

    float magn_out[3];
    _magn_reorient(_sensor.magn, magn_out);
    magn_out[0] *= 1e-6;
    magn_out[1] *= 1e-6;
    magn_out[2] *= 1e-6;

    if(_ros_settings.pub_raw)
    {
        _raw_magn_msg.vector.x = magn_out[0];
        _raw_magn_msg.vector.y = magn_out[1];
        _raw_magn_msg.vector.z = magn_out[2];

        _raw_magn_publisher->publish(_raw_magn_msg);
    }
    if(_ros_settings.pub_static_calib && _static_calibrated)
    {
        _static_calib_magn_msg.header.stamp = _raw_magn_msg.header.stamp;

        _static_calib_magn_msg.vector.x = _static_calib_rot_mat[0][0] * magn_out[0] +
                                          _static_calib_rot_mat[0][1] * magn_out[1] +
                                          _static_calib_rot_mat[0][2] * magn_out[2];
        _static_calib_magn_msg.vector.y = _static_calib_rot_mat[1][0] * magn_out[0] +
                                          _static_calib_rot_mat[1][1] * magn_out[1] +
                                          _static_calib_rot_mat[1][2] * magn_out[2];
        _static_calib_magn_msg.vector.z = _static_calib_rot_mat[2][0] * magn_out[0] +
                                          _static_calib_rot_mat[2][1] * magn_out[1] +
                                          _static_calib_rot_mat[2][2] * magn_out[2];

        _static_calib_magn_publisher->publish(_static_calib_magn_msg);
    }
    if(_ros_settings.pub_calib && _calibrated)
    {
        _calib_magn_msg.header.stamp = _raw_magn_msg.header.stamp;

        _calib_magn_msg.vector.x = _calib_rot_mat[0][0] * magn_out[0] +
                                   _calib_rot_mat[0][1] * magn_out[1] +
                                   _calib_rot_mat[0][2] * magn_out[2];
        _calib_magn_msg.vector.y = _calib_rot_mat[1][0] * magn_out[0] +
                                   _calib_rot_mat[1][1] * magn_out[1] +
                                   _calib_rot_mat[1][2] * magn_out[2];
        _calib_magn_msg.vector.z = _calib_rot_mat[2][0] * magn_out[0] +
                                   _calib_rot_mat[2][1] * magn_out[1] +
                                   _calib_rot_mat[2][2] * magn_out[2];

        _calib_magn_publisher->publish(_calib_magn_msg);
    }
}


/**
 * @brief This function reorients magnetometer axii to match
 *        accelerometer and gyroscope axii. Check ICM20948
 *        manual, page 83.
*/
void ICM20948Reader::_magn_reorient(const float *in, float *out)
{
    out[0] = in[0];
    out[1] = -in[1];
    out[2] = -in[2];
}


void ICM20948Reader::_calib_request()
{
    static bool static_calib_request = true;

    auto request = std::make_shared<road_quality_msgs::srv::GetImuCalib::Request>();

    request->static_calib_only = static_calib_request;
    static_calib_request = !static_calib_request;

    auto result = _calib_client->async_send_request(request, std::bind(&ICM20948Reader::_calib_response_callback,
                                                                       this,
                                                                       std::placeholders::_1));
}


void ICM20948Reader::_calib_response_callback(rclcpp::Client<road_quality_msgs::srv::GetImuCalib>::SharedFuture future)
{
    auto status = future.wait_for(std::chrono::milliseconds(250));
    if(status == std::future_status::ready)
    {
        auto response = future.get();
        if(response->calib_valid && response->calib.child_frame_id == "imu_static_calib" &&
           (_last_static_calib.header.stamp.sec     != response->calib.header.stamp.sec ||
            _last_static_calib.header.stamp.nanosec != response->calib.header.stamp.nanosec))
        {
            _static_calibrated = true;
            _last_static_calib = response->calib;

            tf2::Quaternion q;
            q.setW(_last_static_calib.transform.rotation.w);
            q.setX(_last_static_calib.transform.rotation.x);
            q.setY(_last_static_calib.transform.rotation.y);
            q.setZ(_last_static_calib.transform.rotation.z);
            _static_calib_rot_mat.setRotation(q);

            RCLCPP_INFO(this->get_logger(), "Detected new IMU static calibration with rotation matrix:\n"
                                            "[%.4f  %.4f  %.4f\n"
                                            " %.4f  %.4f  %.4f\n"
                                            " %.4f  %.4f  %.4f]", 
                                            _static_calib_rot_mat[0][0],
                                            _static_calib_rot_mat[0][1],
                                            _static_calib_rot_mat[0][2],
                                            _static_calib_rot_mat[1][0],
                                            _static_calib_rot_mat[1][1],
                                            _static_calib_rot_mat[1][2],
                                            _static_calib_rot_mat[2][0],
                                            _static_calib_rot_mat[2][1],
                                            _static_calib_rot_mat[2][2]);
        }
        else if(response->calib_valid && response->calib.child_frame_id == "imu_calib" &&
                (_last_calib.header.stamp.sec     != response->calib.header.stamp.sec ||
                 _last_calib.header.stamp.nanosec != response->calib.header.stamp.nanosec))
        {
            _calibrated = true;
            _last_calib = response->calib;

            tf2::Quaternion q;
            q.setW(_last_calib.transform.rotation.w);
            q.setX(_last_calib.transform.rotation.x);
            q.setY(_last_calib.transform.rotation.y);
            q.setZ(_last_calib.transform.rotation.z);
            _calib_rot_mat.setRotation(q);

            RCLCPP_INFO(this->get_logger(), "Detected new IMU calibration with rotation matrix:\n"
                                            "[%.4f  %.4f  %.4f\n"
                                            " %.4f  %.4f  %.4f\n"
                                            " %.4f  %.4f  %.4f]", 
                                            _calib_rot_mat[0][0],
                                            _calib_rot_mat[0][1],
                                            _calib_rot_mat[0][2],
                                            _calib_rot_mat[1][0],
                                            _calib_rot_mat[1][1],
                                            _calib_rot_mat[1][2],
                                            _calib_rot_mat[2][0],
                                            _calib_rot_mat[2][1],
                                            _calib_rot_mat[2][2]);
        }
        else if(!response->calib_valid && response->calib.child_frame_id == "imu_static_calib")
        {
            _static_calibrated = false;
            RCLCPP_INFO(this->get_logger(), "Transform %s <- %s not yet ready...", response->calib.child_frame_id.c_str(),
                                                                                   response->calib.header.frame_id.c_str());
        }
        else if(!response->calib_valid && response->calib.child_frame_id == "imu_calib")
        {
            _calibrated = false;
            RCLCPP_INFO(this->get_logger(), "Transform %s <- %s not yet ready...", response->calib.child_frame_id.c_str(),
                                                                                   response->calib.header.frame_id.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "IMU get-calibration service did not respond in time");
    }
}
