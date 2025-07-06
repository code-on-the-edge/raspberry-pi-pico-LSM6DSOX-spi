#ifndef LSM6DSOX_HPP
#define LSM6DSOX_HPP

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "math.h"
#include "spi.hpp"
#include "mpu.hpp"

class LSM6DSOX {
    public:
// Identification
const uint8_t WHO_AM_I        = 0x0F;

// Control Registers
    const uint8_t CTRL1_XL        = 0x10; // Accelerometer control
    const uint8_t CTRL2_G         = 0x11; // Gyroscope control
    const uint8_t CTRL3_C         = 0x12; // Common control (BDU, IF_INC, etc.)
    const uint8_t CTRL4_C         = 0x13;
    const uint8_t CTRL5_C         = 0x14;
    const uint8_t CTRL6_C         = 0x15; // Gyro LPF
    const uint8_t CTRL7_G         = 0x16; // Gyro settings
    const uint8_t CTRL8_XL        = 0x17; // Accel filter settings
    const uint8_t CTRL9_XL        = 0x18;
    const uint8_t CTRL10_C        = 0x19;

    // Status & Interrupts
    const uint8_t STATUS_REG      = 0x1E;
    const uint8_t INT1_CTRL       = 0x0D;
    const uint8_t INT2_CTRL       = 0x0E;

    // Output Registers (Gyroscope)
    const uint8_t OUTX_L_G        = 0x22;
    const uint8_t OUTX_H_G        = 0x23;
    const uint8_t OUTY_L_G        = 0x24;
    const uint8_t OUTY_H_G        = 0x25;
    const uint8_t OUTZ_L_G        = 0x26;
    const uint8_t OUTZ_H_G        = 0x27;

    // Output Registers (Accelerometer)
    const uint8_t OUTX_L_A        = 0x28;
    const uint8_t OUTX_H_A        = 0x29;
    const uint8_t OUTY_L_A        = 0x2A;
    const uint8_t OUTY_H_A        = 0x2B;
    const uint8_t OUTZ_L_A        = 0x2C;
    const uint8_t OUTZ_H_A        = 0x2D;

    // FIFO (optional)
    const uint8_t FIFO_CTRL1      = 0x07;
    const uint8_t FIFO_CTRL2      = 0x08;
    const uint8_t FIFO_CTRL3      = 0x09;
    const uint8_t FIFO_CTRL4      = 0x0A;
    const uint8_t FIFO_STATUS1    = 0x3A;
    const uint8_t FIFO_STATUS2    = 0x3B;
    const uint8_t FIFO_DATA_OUT_L = 0x3E;
    const uint8_t FIFO_DATA_OUT_H = 0x3F;


        LSM6DSOX(Spi *spi_ptr);
        int LSM6DSOX_init();
        void LSM6DSOX_read_raw();


    private:
        Spi *spi_ptr;
        uint8_t buf_raw[12];
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        float accel[3];
        float gyro[3];
        float accel_filtered[3];
        float gyro_filtered[3];
        float ned_accel_n, ned_accel_e, ned_accel_d;
        float ned_gyro_n, ned_gyro_e, ned_gyro_d;
};
#endif