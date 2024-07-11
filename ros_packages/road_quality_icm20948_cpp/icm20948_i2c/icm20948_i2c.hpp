#ifndef ICM20948_I2C_HPP
#define ICM20948_I2C_HPP

#include <cstdint>
#include <mraa/common.hpp>
#include <mraa/i2c.hpp>

#include "icm20948_defs.hpp"
#include "icm20948_utils.hpp"

class ICM20948_i2c
{
    private:
        mraa::I2c _i2c;
        unsigned _i2c_bus, _i2c_address;
        uint8_t _current_bank;
        float _accel_scale_factor, _gyro_scale_factor, _magn_scale_factor;

        bool _write_byte(const uint8_t bank, const uint8_t reg, const uint8_t byte);
        bool _read_byte(const uint8_t bank, const uint8_t reg, uint8_t &byte);
        bool _write_bit(const uint8_t bank, const uint8_t reg, const uint8_t bit_pos, const bool bit);
        bool _read_bit(const uint8_t bank, const uint8_t reg, const uint8_t bit_pos, bool &bit);
        bool _read_block_bytes(const uint8_t bank, const uint8_t start_reg, uint8_t *bytes, const int length);
        bool _write_mag_byte(const uint8_t mag_reg, const uint8_t byte);
        bool _read_mag_byte(const uint8_t mag_reg, uint8_t &byte);

        bool _set_bank(uint8_t bank);
        bool _set_accel_sample_rate_div();
        bool _set_accel_range_dlpf();
        bool _set_gyro_sample_rate_div();
        bool _set_gyro_range_dlpf();
        
        bool _magnetometer_init();
        bool _magnetometer_enable();
        bool _magnetometer_set_mode();
        bool _magnetometer_configured();
        bool _magnetometer_set_readout();

        bool _chip_i2c_master_reset();

    public:
        // Linear acceleration in m/s^2
        float accel[3];
        // Angular velocities in rad/s
        float gyro[3];
        // Magnetic field strength in uTesla
        float magn[3];

        icm20948::settings settings;

        ICM20948_i2c(unsigned i2c_bus, unsigned i2c_address = ICM20948_I2C_ADDR, icm20948::settings settings = icm20948::settings());

        bool init();
        bool reset();
        bool wake();
        bool set_settings();

        bool read_accel_gyro();
        bool read_magn();
};

#endif