#ifndef PI_GPIO_TYPES_HPP
#define PI_GPIO_TYPES_HPP

#include <map>

#include <pigpiod_if2.h>

#include "moving_average.hpp"

typedef enum {PERIODIC=0, RISING_FALLING, RISING, FALLING} publish_mode_t;

const std::map <publish_mode_t, std::string> publish_mode_str = {{PERIODIC, "periodic"},
                                                                 {RISING_FALLING, "rising/falling"},
                                                                 {RISING, "rising"},
                                                                 {FALLING, "falling"}};

const std::map <unsigned, std::string> pull_up_down_str = {{PI_PUD_UP, "pull_up"},
                                                           {PI_PUD_DOWN, "pull_down"},
                                                           {PI_PUD_OFF, "off"}};


typedef struct output_pin_t
{
    unsigned pin;
    bool state;
} output_pin_t;


typedef struct input_pin_t
{
    unsigned pin;
    unsigned pull_up_down;
    bool state;
    publish_mode_t publish_mode;
    MovingAverage<bool> filtered_state;
    
    input_pin_t(int window_size) : filtered_state(window_size) {};
} input_pin_t;


typedef struct pwm_pin_t
{
    unsigned pin;
    unsigned frequency;
    unsigned duty;
} pwm_pin_t;

#endif