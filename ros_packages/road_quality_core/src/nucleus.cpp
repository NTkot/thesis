#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "nucleus.hpp"
#include "nucleus_types.hpp"

Nucleus::Nucleus() : Node("nucleus")
{
    this->_init_tf();

    this->_init_button_functions();
}

void Nucleus::_init_tf()
{
    this->_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->_tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->_tf_buffer);
    this->_imu_calib_srv  = this->create_service<road_quality_msgs::srv::GetImuCalib>
                         (
                            "get_imu_calib", 
                            std::bind(&Nucleus::_imu_calib_srv_callback, this, std::placeholders::_1, std::placeholders::_2)
                         );
    RCLCPP_INFO(this->get_logger(), "Initialized '/get_imu_calib' service");
}

void Nucleus::_init_button_functions()
{
    this->_init_rosbag_on_button();

    this->_init_imu_static_calib_on_button();

    this->_init_imu_dyn_calib_on_button();
}

void Nucleus::_init_rosbag_on_button()
{
    this->declare_parameter("rosbag_on_button_enable", true);
    this->declare_parameter("rosbag_on_button_input_pin", 25);
    this->declare_parameter("rosbag_on_button_led_pin", 26);
    this->declare_parameter("rosbag_on_button_output_path", "/mnt/usb/bag_db");

    this->_rosbag_record.enable = this->get_parameter("rosbag_on_button_enable").as_bool();
    if(this->_rosbag_record.enable)
    {
        rclcpp::ServicesQoS service_qos;

        this->_rosbag_record.recording = false;
        this->_rosbag_record.recording_pid = -1;

        this->_rosbag_record.button_pin = this->get_parameter("rosbag_on_button_input_pin").as_int();
        this->_rosbag_record.indicator_pin = this->get_parameter("rosbag_on_button_led_pin").as_int();
        this->_rosbag_record.output_path = this->get_parameter("rosbag_on_button_output_path").as_string();

        std::filesystem::path rosbag_output_path(this->_rosbag_record.output_path);
        if(!std::filesystem::exists(rosbag_output_path))
        {
            RCLCPP_WARN(this->get_logger(), "Could not find preferred rosbag output path, defaulting to '/home/rpi4/bag_db'");
            this->_rosbag_record.output_path = "/home/rpi4/bag_db";
        }
        // std::string mkdir_data_path = "mkdir -p " + this->_rosbag_record.output_path;
        // if(system(mkdir_data_path.c_str()) != 0)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Could not create rosbag storage folder");
        // }

        this->_rosbag_record.button_pin_subscription =
            this->create_subscription<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/inputs/pin") + std::to_string(this->_rosbag_record.button_pin),
                service_qos,
                std::bind(&Nucleus::_button_functions_callback, this, std::placeholders::_1)
            );
        this->_rosbag_record.indicator_pin_publisher =
            this->create_publisher<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/outputs/pin") + std::to_string(this->_rosbag_record.indicator_pin),
                service_qos
            );
        
        RCLCPP_INFO(this->get_logger(), "Initialized rosbag on button press functionality");
    }
}

void Nucleus::_init_imu_static_calib_on_button()
{
    this->declare_parameter("imu_static_calib_on_button_enable", true);
    this->declare_parameter("imu_static_calib_on_button_input_pin", 16);
    this->declare_parameter("imu_static_calib_on_button_led_pin", 19);

    this->_imu_static_calib.enable = this->get_parameter("imu_static_calib_on_button_enable").as_bool();
    if(this->_imu_static_calib.enable)
    {
        rclcpp::ServicesQoS service_qos;

        this->_imu_static_calib.calibrating = false;
        this->_imu_static_calib.calibrating_pid = -1;

        this->_imu_static_calib.button_pin = this->get_parameter("imu_static_calib_on_button_input_pin").as_int();
        this->_imu_static_calib.indicator_pin = this->get_parameter("imu_static_calib_on_button_led_pin").as_int();

        this->_imu_static_calib.button_pin_subscription =
            this->create_subscription<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/inputs/pin") + std::to_string(this->_imu_static_calib.button_pin),
                service_qos,
                std::bind(&Nucleus::_button_functions_callback, this, std::placeholders::_1)
            );
        this->_imu_static_calib.indicator_pin_publisher =
            this->create_publisher<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/outputs/pin") + std::to_string(this->_imu_static_calib.indicator_pin),
                service_qos
            );

        RCLCPP_INFO(this->get_logger(), "Initialized IMU static calibration on button press functionality");
    }
}

void Nucleus::_init_imu_dyn_calib_on_button()
{
    this->declare_parameter("imu_dyn_calib_on_button_enable", true);
    this->declare_parameter("imu_dyn_calib_on_button_input_pin", 20);
    this->declare_parameter("imu_dyn_calib_on_button_led_pin", 6);

    this->_imu_dyn_calib.enable = this->get_parameter("imu_dyn_calib_on_button_enable").as_bool();
    if(this->_imu_dyn_calib.enable)
    {
        rclcpp::ServicesQoS service_qos;

        this->_imu_dyn_calib.calibrating = false;
        this->_imu_dyn_calib.calibrating_pid = -1;

        this->_imu_dyn_calib.button_pin = this->get_parameter("imu_dyn_calib_on_button_input_pin").as_int();
        this->_imu_dyn_calib.indicator_pin = this->get_parameter("imu_dyn_calib_on_button_led_pin").as_int();

        this->_imu_dyn_calib.button_pin_subscription =
            this->create_subscription<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/inputs/pin") + std::to_string(this->_imu_dyn_calib.button_pin),
                service_qos,
                std::bind(&Nucleus::_button_functions_callback, this, std::placeholders::_1)
            );
        this->_imu_dyn_calib.indicator_pin_publisher =
            this->create_publisher<road_quality_msgs::msg::PinState>
            (
                std::string("/gpio/outputs/pin") + std::to_string(this->_imu_dyn_calib.indicator_pin),
                service_qos
            );

        RCLCPP_INFO(this->get_logger(), "Initialized IMU dynamic calibration on button press functionality");
    }
}

void Nucleus::_button_functions_callback(const road_quality_msgs::msg::PinState::SharedPtr msg)
{
    if(this->_rosbag_record.enable && msg->pin == this->_rosbag_record.button_pin)
    {
        if(!this->_rosbag_record.recording)
        {
            std::string current_date = this->_get_current_date();
            std::string data_path = this->_rosbag_record.output_path + "/data_" + current_date;
            std::string bag_name = data_path + "/rosbag2_" + current_date;

            char* bag_name_c_str = new char[bag_name.size() + 1];
            strcpy(bag_name_c_str, bag_name.c_str());
            char *const cmd[] = {"ros2", "bag", "record", "-a", "-x", "^/image_raw$|^/image_raw/compressedDepth$", "-o", bag_name_c_str, NULL};

            std::string mkdir_data_path = "mkdir -p " + data_path;
            if(system(mkdir_data_path.c_str()) != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not create data storage folder");
                delete[] bag_name_c_str;
                return;
            }

            this->_rosbag_record.recording_pid = this->_launch_process(cmd);
            delete[] bag_name_c_str;
            if(this->_rosbag_record.recording_pid > 0)
            {
                try
                {
                    std::string road_quality_gpio_share_path = ament_index_cpp::get_package_share_directory("road_quality_icm20948_cpp");
                    std::string cp_cmd = "cp " + road_quality_gpio_share_path + "/imu.yaml " + data_path + "/";
                    if(system(cp_cmd.c_str()) != 0)
                    {
                        RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not copy config file for road_quality_icm20948_cpp, skipping...");
                    }
                }
                catch(const ament_index_cpp::PackageNotFoundError &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not find share path for road_quality_icm20948_cpp, skipping...");
                }

                try
                {
                    std::string road_quality_mtk3339_cpp_share_path = ament_index_cpp::get_package_share_directory("road_quality_mtk3339_cpp");
                    std::string cp_cmd = "cp " + road_quality_mtk3339_cpp_share_path + "/gps.yaml " + data_path + "/";
                    if(system(cp_cmd.c_str()) != 0)
                    {
                        RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not copy config file for road_quality_mtk3339_cpp, skipping...");
                    }
                }
                catch(const ament_index_cpp::PackageNotFoundError &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not find share path for road_quality_mtk3339_cpp, skipping...");
                }

                try
                {
                    std::string road_quality_gpio_share_path = ament_index_cpp::get_package_share_directory("road_quality_gpio");
                    std::string cp_cmd = "cp " + road_quality_gpio_share_path + "/gpio.yaml " + data_path + "/";
                    if(system(cp_cmd.c_str()) != 0)
                    {
                        RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not copy config file for road_quality_gpio, skipping...");
                    }
                }
                catch(const ament_index_cpp::PackageNotFoundError &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Rosbag recorder: could not find share path for road_quality_gpio, skipping...");
                }

                road_quality_msgs::msg::PinState pub_msg;
                
                pub_msg.header.stamp = rclcpp::Node::now();
                pub_msg.pin = (uint8_t)this->_rosbag_record.indicator_pin;
                pub_msg.state = true;
                this->_rosbag_record.indicator_pin_publisher->publish(pub_msg);

                this->_rosbag_record.recording = true;
            }
        }
        else
        {
            if(this->_kill_process(this->_rosbag_record.recording_pid))
            {
                road_quality_msgs::msg::PinState pub_msg;

                pub_msg.header.stamp = rclcpp::Node::now();
                pub_msg.pin = (uint8_t)this->_rosbag_record.indicator_pin;
                pub_msg.state = false;
                this->_rosbag_record.indicator_pin_publisher->publish(pub_msg);

                this->_rosbag_record.recording_pid = -1;
                this->_rosbag_record.recording = false;
            }
        }
    }
    else if(this->_imu_static_calib.enable && msg->pin == this->_imu_static_calib.button_pin)
    {
        static auto last_pressed_time = std::chrono::high_resolution_clock::now();
        if(!this->_imu_static_calib.calibrating)
        {
            if(msg->state == true)
            {
                last_pressed_time = std::chrono::high_resolution_clock::now();
            }
            else
            {
                int64_t pressed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - 
                                                                                      last_pressed_time).count();
                if(pressed_time_ms > 3000)
                {
                    char *const cmd[] = {"ros2", "run", "road_quality_calib", "imu_static_calib_reverse_parking", NULL};
                    this->_imu_static_calib.calibrating_pid = this->_launch_process(cmd);
                    this->_imu_static_calib.reverse_parking = true;
                }
                else
                {
                    char *const cmd[] = {"ros2", "run", "road_quality_calib", "imu_static_calib", NULL};
                    this->_imu_static_calib.calibrating_pid = this->_launch_process(cmd);
                    this->_imu_static_calib.reverse_parking = false;
                }

                if(this->_imu_static_calib.calibrating_pid > 0)
                {
                    road_quality_msgs::msg::PinState pub_msg;

                    pub_msg.header.stamp = rclcpp::Node::now();
                    pub_msg.pin = (uint8_t)this->_imu_static_calib.indicator_pin;
                    pub_msg.state = true;
                    this->_imu_static_calib.indicator_pin_publisher->publish(pub_msg);

                    this->_imu_static_calib.calibrating = true;

                    this->_imu_static_calib.check_pid_timer = this->create_wall_timer(std::chrono::milliseconds(250), 
                                                                                      std::bind(&Nucleus::_check_imu_static_calib_process, this));
                }
            }
        }
    }
    else if(this->_imu_dyn_calib.enable && msg->pin == this->_imu_dyn_calib.button_pin)
    {
        if(!this->_imu_dyn_calib.calibrating)
        {
            char *const cmd[] = {"ros2", "run", "road_quality_calib", "imu_dyn_calib_brake", NULL};

            this->_imu_dyn_calib.calibrating_pid = this->_launch_process(cmd);
            if(this->_imu_dyn_calib.calibrating_pid > 0)
            {
                road_quality_msgs::msg::PinState pub_msg;
                
                pub_msg.header.stamp = rclcpp::Node::now();
                pub_msg.pin = (uint8_t)this->_imu_dyn_calib.indicator_pin;
                pub_msg.state = true;
                this->_imu_dyn_calib.indicator_pin_publisher->publish(pub_msg);

                this->_imu_dyn_calib.calibrating = true;

                this->_imu_dyn_calib.check_pid_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                                                               std::bind(&Nucleus::_check_imu_dyn_calib_process, this));
            }
        }
    }
}

void Nucleus::_imu_calib_srv_callback(const std::shared_ptr<road_quality_msgs::srv::GetImuCalib::Request>  request,
                                      const std::shared_ptr<road_quality_msgs::srv::GetImuCalib::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "Get IMU calibration service: called");

    std::string child_frame;
    if(request->static_calib_only)
    {

        child_frame = "imu_static_calib";
    }
    else
    {
        child_frame = "imu_calib";
    }
    
    try 
    {
        geometry_msgs::msg::TransformStamped static_calib;
        static_calib = this->_tf_buffer->lookupTransform("imu",
                                                         child_frame,
                                                         tf2::TimePointZero);

        response->calib_valid = true;
        response->calib = static_calib;
        // RCLCPP_INFO(this->get_logger(), "Get IMU calibration service: transform '%s' <- 'imu' sent", child_frame.c_str());
    }
    catch(const tf2::TransformException &ex)
    {
        response->calib_valid = false;
        // RCLCPP_INFO(this->get_logger(), "Get IMU calibration service: transform '%s' <- 'imu' not available", child_frame.c_str());
    }
}

void Nucleus::_check_imu_static_calib_process()
{
    int status;
    pid_t return_pid;
    return_pid = waitpid(this->_imu_static_calib.calibrating_pid, &status, WNOHANG);

    if(return_pid == 0)
    {
        if(this->_imu_static_calib.reverse_parking)
        {
            static bool prev_state = true;
            road_quality_msgs::msg::PinState pub_msg;

            pub_msg.header.stamp = rclcpp::Node::now();
            pub_msg.pin = (uint8_t)this->_imu_static_calib.indicator_pin;
            pub_msg.state = !prev_state;
            prev_state = pub_msg.state;
            this->_imu_static_calib.indicator_pin_publisher->publish(pub_msg);
        }
        // RCLCPP_INFO(this->get_logger(), "IMU static calibration process still running");
    }
    else
    {
        road_quality_msgs::msg::PinState pub_msg;

        pub_msg.header.stamp = rclcpp::Node::now();
        pub_msg.pin = (uint8_t)this->_imu_static_calib.indicator_pin;
        pub_msg.state = false;
        this->_imu_static_calib.indicator_pin_publisher->publish(pub_msg);

        this->_imu_static_calib.calibrating = false;
        this->_imu_static_calib.calibrating_pid = -1;
        this->_imu_static_calib.check_pid_timer->cancel();

        RCLCPP_INFO(this->get_logger(), "IMU static calibration process has finished, turning off indicator");
    }
}

void Nucleus::_check_imu_dyn_calib_process()
{
    int status;
    pid_t return_pid;
    return_pid = waitpid(this->_imu_dyn_calib.calibrating_pid, &status, WNOHANG);

    if(return_pid == 0)
    {
        // RCLCPP_INFO(this->get_logger(), "IMU dynamic calibration process still running");
    }
    else
    {
        road_quality_msgs::msg::PinState pub_msg;

        pub_msg.header.stamp = rclcpp::Node::now();
        pub_msg.pin = (uint8_t)this->_imu_dyn_calib.indicator_pin;
        pub_msg.state = false;
        this->_imu_dyn_calib.indicator_pin_publisher->publish(pub_msg);

        this->_imu_dyn_calib.calibrating = false;
        this->_imu_dyn_calib.calibrating_pid = -1;
        this->_imu_dyn_calib.check_pid_timer->cancel();

        RCLCPP_INFO(this->get_logger(), "IMU dynamic calibration process has finished, turning off indicator");
    }
}

pid_t Nucleus::_launch_process(char *const cmd[])
{
    std::string cmd_str;
    for(int i = 0; cmd[i] != NULL; i++)
        cmd_str += std::string(cmd[i]) + " ";
    cmd_str.pop_back();

    pid_t ret = fork();
    if(ret == 0)
    {
        int res = execvp(cmd[0], cmd);
        if(res < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create process '%s' with error: %s", cmd_str.c_str(), 
                                                                                             strerror(errno));
            exit(1);
        }
    }
    else if(ret > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Created process '%s' with PID %d", cmd_str.c_str(), ret);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Fork FAILED to create process '%s' with error: %s", cmd_str.c_str(), 
                                                                                              strerror(errno));
    }

    return ret;
}

bool Nucleus::_kill_process(pid_t pid)
{
    int res = 0;
    int res_kill = 0;

    // shell command that sends SIGINT to all child processes from pid argument if parent exists
    std::string kill_child_cmd = "kill -0 " + std::to_string(pid) + "> /dev/null 2>&1; "
                                 "if [ \"$?\" = 0 ]; "
                                 "then "
                                 "for child_pid in $(pgrep -P " + std::to_string(pid) + "); do "
                                 "kill -2 $child_pid; "
                                 "done; "
                                 "else "
                                 "exit $?; "
                                 "fi";

    // system() returns last returned exit code ($?)
    res += system(kill_child_cmd.c_str());
    if(res != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not kill child processes of PID: %d", pid);
    }

    // Kill main process
    res_kill = kill(pid, SIGINT);
    if(res_kill != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not kill process with PID: %d", pid);
    }
    // kill() returns 0 on success, -1 on failure (that's why we use -=)
    res -= res_kill;

    if(res == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Killed process and its children with PID: %d", pid);
    }

    return (res == 0);
}

std::string Nucleus::_get_current_date()
{
    auto now = std::chrono::system_clock::now();

    std::time_t current_time = std::chrono::system_clock::to_time_t(now);

    std::tm* local_time = std::localtime(&current_time);

    std::stringstream date_stream;
    date_stream << std::setfill('0');

    date_stream << std::setw(2) << local_time->tm_mday << '_';
    date_stream << std::setw(2) << (local_time->tm_mon + 1) << '_';
    date_stream << (local_time->tm_year + 1900) << "__";
    date_stream << std::setw(2) << local_time->tm_hour << '_';
    date_stream << std::setw(2) << local_time->tm_min << '_';
    date_stream << std::setw(2) << local_time->tm_sec;

    return date_stream.str();
}
