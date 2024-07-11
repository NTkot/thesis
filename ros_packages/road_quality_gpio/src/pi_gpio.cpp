#include <chrono>
#include <thread>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "pi_gpio.hpp"
#include "pi_gpio_types.hpp"
#include "road_quality_msgs/msg/pin_state.hpp"
#include "road_quality_msgs/msg/pin_pwm_state.hpp"

PiGpio::PiGpio(const std::string &path_to_config) : Node("gpio_handler"), _out_pins(), _in_pins(), _publishers(), _subscriptions()
{
    this->pigpio_ret = pigpio_start(NULL, NULL);
    if(pigpio_ret < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize PiGPIO library");
        exit(1);
    }

    this->_parse_config_file(path_to_config);

    this->_init_pins();

    this->_init_ros();
}

PiGpio::~PiGpio()
{
    pigpio_stop(this->pigpio_ret);
}

void PiGpio::_outputs_callback(const road_quality_msgs::msg::PinState::SharedPtr msg)
{
    for(std::vector<output_pin_t>::iterator iter = this->_out_pins.begin(); iter < _out_pins.end(); iter++)
    {
        if((iter->pin == msg->pin) && (iter->state != msg->state))
        {
            if(gpio_write(this->pigpio_ret, iter->pin, msg->state) != 0)
                RCLCPP_ERROR(this->get_logger(), "Could not write GPIO%u (new state: %u, old state %u)", iter->pin, msg->state, iter->state);
            else
                iter->state = msg->state;
        }
    }
}

void PiGpio::_inputs_timer_callback()
{
    for(std::vector<input_pin_t>::iterator iter = this->_in_pins.begin(); iter < _in_pins.end(); iter++)
    {
        int state = gpio_read(this->pigpio_ret, iter->pin);
        if(state < 0)
            RCLCPP_ERROR(this->get_logger(), "Could not read GPIO%u", iter->pin);
        else
        {
            // Pass gpioRead output through pin filter
            iter->filtered_state.insert((bool)state);
            state = (iter->filtered_state.value() > 0.5) ? 1 : 0;

            // At this point, 
            // 'state' contains new state, 
            // 'iter->state' contains old state

            // Check for rising, falling or any edge detection
            bool state_changed = (((bool)state) != iter->state);
            bool state_rising_occured = ((state == 1) && (iter->state == false));
            bool state_falling_occured = ((state == 0) && (iter->state == true));

            // Update old pin state with new state
            iter->state = (bool)state;

            if((iter->publish_mode == PERIODIC) || 
               (iter->publish_mode == RISING_FALLING && state_changed) ||
               (iter->publish_mode == RISING && state_rising_occured) ||
               (iter->publish_mode == FALLING && state_falling_occured))
            {
                road_quality_msgs::msg::PinState msg;
                msg.header.stamp = rclcpp::Node::now();
                msg.pin = iter->pin;
                msg.state = iter->state;
                this->_publishers[iter->pin]->publish(msg);
            }
        }
    }
}

void PiGpio::_pwm_callback(const road_quality_msgs::msg::PinPwmState::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Detected incoming PWM message for pin %u", msg->pin);
    for(std::vector<pwm_pin_t>::iterator iter = this->_pwm_pins.begin(); iter < _pwm_pins.end(); iter++)
    {
        if((iter->pin == msg->pin) && ((iter->frequency != msg->frequency) || (iter->duty != msg->duty)))
        {
            RCLCPP_INFO(this->get_logger(), "Pin matches configured PWM pin %u", iter->pin);
            if(hardware_PWM(this->pigpio_ret, iter->pin, msg->frequency, msg->duty) != 0)
                RCLCPP_ERROR(this->get_logger(), "Could not write GPIO%u (new freq: %u, new duty: %u, old freq %u, old duty: %u)", iter->pin, msg->frequency, msg->duty, iter->frequency, iter->duty);
            else
            {
                iter->frequency = msg->frequency;
                iter->duty = msg->duty;
                RCLCPP_INFO(this->get_logger(), "Changed frequency to %u and duty cycle to %u", iter->frequency, iter->duty);
            }
        }
    }
}

void PiGpio::_parse_config_file(const std::string &path_to_config)
{
    this->_config = YAML::LoadFile(path_to_config.c_str());

    for(YAML::const_iterator it = this->_config.begin(); it != this->_config.end(); ++it)
    {
        if(it->first.as<std::string>() == "input_read_period_ms")
        {
            this->_timer_period_ms = it->second.as<float>();
            continue;
        }

        try
        {
            it->first.as<unsigned>();
            it->second["mode"].as<std::string>();
        }
        catch(const YAML::TypedBadConversion<unsigned>& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "First level entry in YAML config is not a non-negative integer");
            throw(ex);
        }
        catch(const YAML::InvalidNode& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Second level entry in YAML config does not contain 'mode'");
            throw(ex);
        }


        if(it->second["mode"].as<std::string>() == "input")
        {
            input_pin_t new_pin(it->second["filter_size"].as<int>());

            new_pin.pin = it->first.as<unsigned>();

            std::string pud = it->second["pud"].as<std::string>();
            if(pud == "up")
            {
                new_pin.pull_up_down = PI_PUD_UP;
                new_pin.state = true;
            }
            else if(pud == "down")
            {
                new_pin.pull_up_down = PI_PUD_DOWN;
                new_pin.state = false;
            }
            else
            {
                new_pin.pull_up_down = PI_PUD_OFF;
            }


            std::string publish_mode = it->second["publish_mode"].as<std::string>();
            if(publish_mode == "rising_falling")
                new_pin.publish_mode = RISING_FALLING;
            else if(publish_mode == "rising")
                new_pin.publish_mode = RISING;
            else if(publish_mode == "falling")
                new_pin.publish_mode = FALLING;
            else
                new_pin.publish_mode = PERIODIC;
            
            this->_in_pins.push_back(new_pin);
        }
        else if(it->second["mode"].as<std::string>() == "output")
        {
            output_pin_t new_pin;
            new_pin.pin   = it->first.as<unsigned>();
            new_pin.state = (bool)it->second["init"].as<int>();
            this->_out_pins.push_back(new_pin);
        }
        else if(it->second["mode"].as<std::string>() == "pwm")
        {
            pwm_pin_t new_pin;
            new_pin.pin       = it->first.as<unsigned>();
            new_pin.frequency = it->second["init_freq"].as<unsigned>();
            new_pin.duty      = (unsigned)(it->second["init_duty"].as<float>() * 1000000.0);
            this->_pwm_pins.push_back(new_pin);
        }
    }
}

void PiGpio::_init_pins()
{
    for(std::vector<output_pin_t>::iterator iter = this->_out_pins.begin(); iter < _out_pins.end(); iter++)
    {
        if(set_mode(this->pigpio_ret, iter->pin, PI_OUTPUT) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not set output mode to GPIO%u", iter->pin);
            PiGpioException ex("'set_mode' exception occured");
            throw(ex);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        if(gpio_write(this->pigpio_ret, iter->pin, (unsigned)iter->state) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not write initial state to GPIO%u", iter->pin);
            PiGpioException ex("'gpio_write' exception occured");
            throw(ex);
        }

        RCLCPP_INFO(this->get_logger(), "Initialized GPIO%u as output (init state: %u)", iter->pin, 
                                                                               (unsigned)iter->state);
    }


    for(std::vector<input_pin_t>::iterator iter = this->_in_pins.begin(); iter < _in_pins.end(); iter++)
    {
        if(set_mode(this->pigpio_ret, iter->pin, PI_INPUT) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not set input mode to GPIO%u", iter->pin);
            PiGpioException ex("'set_mode' exception occured");
            throw(ex);
        }

        if(set_pull_up_down(this->pigpio_ret, iter->pin, iter->pull_up_down) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not set pull up/down mode to GPIO%u", iter->pin);
            PiGpioException ex("'set_pull_up_down' exception occured");
            throw(ex);
        }

        RCLCPP_INFO(this->get_logger(), "Initialized GPIO%u as input (pull-up/down: %s, "
                                                                     "filter window size: %u, "
                                                                     "publish_method: %s)", iter->pin, 
                                                                                            pull_up_down_str.at(iter->pull_up_down).c_str(), 
                                                                                            iter->filtered_state.size(),
                                                                                            publish_mode_str.at(iter->publish_mode).c_str());
    }


    for(std::vector<pwm_pin_t>::iterator iter = this->_pwm_pins.begin(); iter < _pwm_pins.end(); iter++)
    {
        if(hardware_PWM(this->pigpio_ret, iter->pin, iter->frequency, iter->duty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not set PWM to GPIO%u", iter->pin);
            PiGpioException ex("'hardware_PWM' exception occured");
            throw(ex);
        }

        RCLCPP_INFO(this->get_logger(), "Initialized GPIO%u as PWM pin (Init frequency: %uHz, "
                                                                       "Init duty cycle: %.2f%%)", iter->pin,
                                                                                                   iter->frequency,
                                                                                                   ((float)iter->duty) / 10000.0);
    }
}

void PiGpio::_init_ros()
{
    rclcpp::ServicesQoS service_qos;
    rclcpp::SensorDataQoS sensor_data_qos;

    for(std::vector<output_pin_t>::iterator iter = this->_out_pins.begin(); iter < _out_pins.end(); iter++)
    {
        this->_subscriptions.insert
        (
            std::pair
            <
                unsigned,
                rclcpp::Subscription<road_quality_msgs::msg::PinState>::SharedPtr
            >
            (
                iter->pin,
                this->create_subscription<road_quality_msgs::msg::PinState>
                (
                    std::string("/gpio/outputs/pin") + std::to_string(iter->pin), 
                    service_qos, 
                    std::bind(&PiGpio::_outputs_callback, this, std::placeholders::_1)
                )
            )
        );
    }

    for(std::vector<input_pin_t>::iterator iter = this->_in_pins.begin(); iter < _in_pins.end(); iter++)
    {
        this->_publishers.insert
        (
            std::pair
            <
                unsigned, 
                rclcpp::Publisher<road_quality_msgs::msg::PinState>::SharedPtr
            >
            (
                iter->pin,
                this->create_publisher<road_quality_msgs::msg::PinState>
                (
                    std::string("/gpio/inputs/pin") + std::to_string(iter->pin), 
                    service_qos
                )
            )
        );
    }

    for(std::vector<pwm_pin_t>::iterator iter = this->_pwm_pins.begin(); iter < _pwm_pins.end(); iter++)
    {
        this->_pwm_subscriptions.insert
        (
            std::pair
            <
                unsigned,
                rclcpp::Subscription<road_quality_msgs::msg::PinPwmState>::SharedPtr
            >
            (
                iter->pin,
                this->create_subscription<road_quality_msgs::msg::PinPwmState>
                (
                    std::string("/gpio/pwm/pin") + std::to_string(iter->pin), 
                    service_qos, 
                    std::bind(&PiGpio::_pwm_callback, this, std::placeholders::_1)
                )
            )
        );
    }

    this->_timer = this->create_wall_timer(std::chrono::milliseconds(int(this->_timer_period_ms)), std::bind(&PiGpio::_inputs_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Initialized GPIO input read timer (frequency: %.2f)", (1 / this->_timer_period_ms) * 1000);
}
