#ifndef NUCLEUS_TYPES_HPP
#define NUCLEUS_TYPES_HPP

#include "rclcpp/rclcpp.hpp"

#include "road_quality_msgs/msg/pin_state.hpp"

typedef struct {
    bool enable;
    unsigned button_pin;
    unsigned indicator_pin;
    rclcpp::Subscription<road_quality_msgs::msg::PinState>::SharedPtr button_pin_subscription;
    rclcpp::Publisher<road_quality_msgs::msg::PinState>::SharedPtr indicator_pin_publisher;
} button_function;

#endif 