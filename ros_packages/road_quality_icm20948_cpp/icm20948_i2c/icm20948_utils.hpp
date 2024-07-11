#ifndef ICM2094_UTILS_HPP
#define ICM2094_UTILS_HPP

#include <cstdint>
#include "yaml-cpp/yaml.h"

namespace icm20948
{

typedef enum {ACCEL_2G = 0, ACCEL_4G, 
              ACCEL_8G,     ACCEL_16G} accel_scale;
float accel_scale_factor(accel_scale scale);
std::string accel_scale_to_str(accel_scale scale);

typedef enum {ACCEL_DLPF_246HZ = 0, ACCEL_DLPF_246HZ_2,
              ACCEL_DLPF_111_4HZ,   ACCEL_DLPF_50_4HZ,
              ACCEL_DLPF_23_9HZ,    ACCEL_DLPF_11_5HZ,
              ACCEL_DLPF_5_7HZ,     ACCEL_DLPF_473HZ} accel_dlpf_config;
std::string accel_dlpf_config_to_str(accel_dlpf_config config);

typedef struct accel_settings
{
    // Range [0, 4095]
    uint16_t sample_rate_div;

    // Full scale of measurements +/-
    accel_scale scale;

    // Digital Low-Pass Filter enable
    bool dlpf_enable;

    // Digital Low-Pass Filter settings
    accel_dlpf_config dlpf_config;


    accel_settings(uint16_t sample_rate_div      = 0,
                   accel_scale scale             = ACCEL_2G,
                   bool dlpf_enable              = true,
                   accel_dlpf_config dlpf_config = ACCEL_DLPF_246HZ) : sample_rate_div(sample_rate_div),
                                                                       scale(scale),
                                                                       dlpf_enable(dlpf_enable),
                                                                       dlpf_config(dlpf_config) {};
} accel_settings;




typedef enum {GYRO_250DPS = 0, GYRO_500DPS, 
              GYRO_1000DPS,    GYRO_2000DPS} gyro_scale;
float gyro_scale_factor(gyro_scale scale);
std::string gyro_scale_to_str(gyro_scale scale);

typedef enum {GYRO_DLPF_196_6HZ = 0, GYRO_DLPF_151_8HZ,
              GYRO_DLPF_119_5HZ,     GYRO_DLPF_51_2HZ,
              GYRO_DLPF_23_9HZ,      GYRO_DLPF_11_6HZ,
              GYRO_DLPF_5_7HZ,       GYRO_DLPF_361_4HZ} gyro_dlpf_config;
std::string gyro_dlpf_config_to_str(gyro_dlpf_config config);

typedef struct gyro_settings
{
    // Range [0, 250]
    uint8_t sample_rate_div;

    // Full scale of measurements +/-
    gyro_scale scale;

    // Digital Low-Pass Filter enable
    bool dlpf_enable;

    // Digital Low-Pass Filter settings
    gyro_dlpf_config dlpf_config;


    gyro_settings(uint8_t sample_rate_div      = 0,
                  gyro_scale scale             = GYRO_250DPS,
                  bool dlpf_enable             = true,
                  gyro_dlpf_config dlpf_config = GYRO_DLPF_196_6HZ) : sample_rate_div(sample_rate_div),
                                                                      scale(scale),
                                                                      dlpf_enable(dlpf_enable),
                                                                      dlpf_config(dlpf_config) {};
} gyro_settings;




typedef enum {MAGN_SHUTDOWN = 0, MAGN_SINGLE = 1, 
              MAGN_10HZ = 2,     MAGN_20HZ = 4,
              MAGN_50HZ = 6,     MAGN_100HZ = 8, MAGN_SELF_TEST = 16} magn_mode;

typedef struct magn_settings
{
    // Magnetometer operation mode
    magn_mode mode;

    magn_settings(magn_mode mode = MAGN_100HZ) : mode(mode) {};
} magn_settings;
std::string magn_mode_to_str(magn_mode mode);



typedef struct settings
{
    accel_settings accel;
    gyro_settings  gyro;
    magn_settings  magn;


    settings(accel_settings accel = accel_settings(), 
             gyro_settings gyro   = gyro_settings(),
             magn_settings magn   = magn_settings()) : accel(accel), 
                                                       gyro(gyro),
                                                       magn(magn) {};
    settings(YAML::Node config_file_node);
} settings;


}

#endif