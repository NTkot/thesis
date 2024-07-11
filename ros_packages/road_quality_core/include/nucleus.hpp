#ifndef NUCLEUS_HPP
#define NUCLEUS_HPP

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nucleus_types.hpp"
#include "road_quality_msgs/msg/pin_state.hpp"
#include "road_quality_msgs/srv/get_imu_calib.hpp"


class Nucleus : public rclcpp::Node
{
    private:
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
        rclcpp::Service<road_quality_msgs::srv::GetImuCalib>::SharedPtr _imu_calib_srv;

        struct rosbag_record : button_function 
        {
            bool recording;
            pid_t recording_pid;
            std::string output_path;
        } _rosbag_record;

        struct imu_static_calib : button_function 
        {
            bool calibrating;
            pid_t calibrating_pid;
            rclcpp::TimerBase::SharedPtr check_pid_timer;
            bool reverse_parking;
        } _imu_static_calib;

        struct imu_dyn_calib : button_function 
        {
            bool calibrating;
            pid_t calibrating_pid;
            rclcpp::TimerBase::SharedPtr check_pid_timer;
        } _imu_dyn_calib;

        void _init_tf();
        void _init_button_functions();
        void _init_rosbag_on_button();
        void _init_imu_static_calib_on_button();
        void _init_imu_dyn_calib_on_button();

        void _imu_calib_srv_callback(const std::shared_ptr<road_quality_msgs::srv::GetImuCalib::Request>  request,
                                     const std::shared_ptr<road_quality_msgs::srv::GetImuCalib::Response> response);
        void _button_functions_callback(const road_quality_msgs::msg::PinState::SharedPtr msg);

        void _check_imu_static_calib_process();
        void _check_imu_dyn_calib_process();
        
        pid_t _launch_process(char *const cmd[]);
        bool _kill_process(pid_t pid);
        std::string _get_current_date();

    public:
        Nucleus();
};

#endif
