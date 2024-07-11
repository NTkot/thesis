#ifndef PI_GPIO_HPP
#define PI_GPIO_HPP

#include <vector>
#include <map>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "moving_average.hpp"
#include "pi_gpio_types.hpp"
#include "road_quality_msgs/msg/pin_state.hpp"
#include "road_quality_msgs/msg/pin_pwm_state.hpp"


class PiGpio : public rclcpp::Node
{
    private:
        int pigpio_ret;
        YAML::Node _config;

        std::vector<output_pin_t> _out_pins;
        std::vector<input_pin_t> _in_pins;
        std::vector<pwm_pin_t> _pwm_pins;

        std::map<unsigned, rclcpp::Publisher   <road_quality_msgs::msg::PinState>   ::SharedPtr> _publishers;
        std::map<unsigned, rclcpp::Subscription<road_quality_msgs::msg::PinState>   ::SharedPtr> _subscriptions;
        std::map<unsigned, rclcpp::Subscription<road_quality_msgs::msg::PinPwmState>::SharedPtr> _pwm_subscriptions;
        
        rclcpp::TimerBase::SharedPtr _timer;
        float _timer_period_ms;

        void _inputs_timer_callback();
        void _outputs_callback(const road_quality_msgs::msg::PinState::SharedPtr msg);
        void _pwm_callback(const road_quality_msgs::msg::PinPwmState::SharedPtr msg);
        void _parse_config_file(const std::string &path_to_config);
        void _init_pins();
        void _init_ros();

    public:
        PiGpio(const std::string &path_to_config);
        ~PiGpio();
};

class PiGpioException : public std::exception {
    private:
        std::string message;
    public:
        PiGpioException(std::string msg) : message(msg) {};
        const char* what() {return message.c_str();};
};

#endif
