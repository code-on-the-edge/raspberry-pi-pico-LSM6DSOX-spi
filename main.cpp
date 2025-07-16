#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math.h"

#define MISO 16
#define CS 17
#define SCLK 18
#define MOSI 19

#define SPI_PORT spi0

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

uint8_t buf_raw[12];
int16_t accel_raw[3];
int16_t gyro_raw[3];
float accel[3];
float gyro[3];
float accel_filtered[3];
float gyro_filtered[3];
float ned_accel_n, ned_accel_e, ned_accel_d;
float ned_gyro_n, ned_gyro_e, ned_gyro_d;

void Spi_init(spi_inst_t *spi_inst, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs) {
    printf("Spi init...\n\r");
    spi_init(spi_inst, 4000000);  // 4 MHz (super stable)
    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);

    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

    printf("miso: %d, mosi: %d, sclk: %d, cs: %d\n\r", miso, mosi, sclk, cs);
}

// Write 1 byte to the specified register
void reg_write(spi_inst_t *spi, const uint cs, const uint8_t reg, const uint8_t data) {

    uint8_t msg[2];
                
    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x00 | reg;
    msg[1] = data;

    // Write to register
    gpio_put(cs, 0);
    spi_write_blocking(spi, msg, 2);
    gpio_put(cs, 1);
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(spi_inst_t *spi, const uint cs, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Construct message (set ~W bit high)
    // uint8_t msg = 0x80 | (mb << 6) | reg;
    uint8_t msg = 0x80 | reg;

    // Read from register
    gpio_put(cs, 0);
    spi_write_blocking(spi, &msg, 1);
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
    gpio_put(cs, 1);

    return num_bytes_read;
}

int LSM6DSOX_init() {
    uint8_t buf[1];
    int num_bytes_read = 0;

    printf("LSM6DSOX_init: miso: %d, mosi: %d, sclk: %d, cs: %d\n\r", MISO, MOSI, SCLK, CS);

    // WHO_AM_I check
    num_bytes_read = reg_read(SPI_PORT, CS, WHO_AM_I, buf, 1);
    if (buf[0] != 0x6C) {
        printf("WHO_AM_I mismatch: got 0x%02X\n\r", buf[0]);
        return -1;
    }
    printf("WHO_AM_I : 0x%02X\n\r", buf[0]);

    // CTRL3_C: IF_INC=1 (auto-increment), BDU=1 (block data update)
    reg_write(SPI_PORT, CS, CTRL3_C, 0b01000100);
    reg_read(SPI_PORT, CS, CTRL3_C, buf, 1);
    printf("CTRL3_C : 0x%02X\n\r", buf[0]);

    // CTRL1_XL: Accel @ 833 Hz, ±8g
    reg_write(SPI_PORT, CS, CTRL1_XL, 0b10001100);
    reg_read(SPI_PORT, CS, CTRL1_XL, buf, 1);
    printf("CTRL1_XL : 0x%02X\n\r", buf[0]);

    // CTRL2_G: Gyro @ 833 Hz, ±1000 dps
    reg_write(SPI_PORT, CS, CTRL2_G, 0b10001000);
    reg_read(SPI_PORT, CS, CTRL2_G, buf, 1);
    printf("CTRL2_G : 0x%02X\n\r", buf[0]);

    // CTRL6_C: Gyro LPF1 @ ODR/20
    reg_write(SPI_PORT, CS, CTRL6_C, 0b00000010);
    reg_read(SPI_PORT, CS, CTRL6_C, buf, 1);
    printf("CTRL6_C : 0x%02X\n\r", buf[0]);

    // CTRL8_XL: Accel LPF2 @ ODR/20
    reg_write(SPI_PORT, CS, CTRL8_XL, 0b00000010);
    reg_read(SPI_PORT, CS, CTRL8_XL, buf, 1);
    printf("CTRL8_XL : 0x%02X\n\r", buf[0]);

    // CTRL7_G: High-performance gyro
    reg_write(SPI_PORT, CS, CTRL7_G, 0b00000000);
    reg_read(SPI_PORT, CS, CTRL7_G, buf, 1);
    printf("CTRL7_G : 0x%02X\n\r", buf[0]);

    // CTRL5_C: Normal mode
    reg_write(SPI_PORT, CS, CTRL5_C, 0b00000000);
    reg_read(SPI_PORT, CS, CTRL5_C, buf, 1);
    printf("CTRL5_C : 0x%02X\n\r", buf[0]);

    // INT1_CTRL: DRDY accel + gyro on INT1
    reg_write(SPI_PORT, CS, INT1_CTRL, 0b00000011);
    reg_read(SPI_PORT, CS, INT1_CTRL, buf, 1);
    printf("INT1_CTRL : 0x%02X\n\r", buf[0]);

    return 0;
}

void LSM6DSOX_read_raw() {
    constexpr float ACCEL_SENSITIVITY = 0.244f;   // [mg/LSB] for ±8g
    constexpr float GYRO_SENSITIVITY  = 35.0f;     // [mdps/LSB] for ±1000 dps

    int num_bytes_read = reg_read(SPI_PORT, CS, OUTX_L_G, buf_raw, 12);
    if (num_bytes_read != 12) {
        printf("Error: LSM6DSOX_read_raw failed to read 12 bytes\n\r");
        return;
    }

    // Raw gyroscope data
    gyro_raw[0] = (int16_t)(buf_raw[1] << 8 | buf_raw[0]);
    gyro_raw[1] = (int16_t)(buf_raw[3] << 8 | buf_raw[2]);
    gyro_raw[2] = (int16_t)(buf_raw[5] << 8 | buf_raw[4]);

    // Raw accelerometer data
    accel_raw[0] = (int16_t)(buf_raw[7]  << 8 | buf_raw[6]);
    accel_raw[1] = (int16_t)(buf_raw[9]  << 8 | buf_raw[8]);
    accel_raw[2] = (int16_t)(buf_raw[11] << 8 | buf_raw[10]);

    // Convert to physical units: [g] and [dps]
    accel[0] = (accel_raw[0] * ACCEL_SENSITIVITY) / 1000.0f;
    accel[1] = (accel_raw[1] * ACCEL_SENSITIVITY) / 1000.0f;
    accel[2] = (accel_raw[2] * ACCEL_SENSITIVITY) / 1000.0f;

    gyro[0] = (gyro_raw[0] * GYRO_SENSITIVITY) / 1000.0f;
    gyro[1] = (gyro_raw[1] * GYRO_SENSITIVITY) / 1000.0f;
    gyro[2] = (gyro_raw[2] * GYRO_SENSITIVITY) / 1000.0f;

    // printf("accel:%.6f,%.6f,%.6f, gyro:%.6f,%.6f,%.6f\n\r", 
    //     accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

    // printf("%.6f,%.6f,%.6f\n\r", 
    //    ned_accel_n, ned_accel_e, ned_accel_d);

    // printf("%.6f,%.6f,%.6f\n\r", 
    //    ned_gyro_n, ned_gyro_e, ned_gyro_d);

    constexpr float alpha = 0.9f;  // smoothing factor: smaller = smoother

    for (int i = 0; i < 3; ++i) {
        accel_filtered[i] = alpha * accel[i] + (1.0f - alpha) * accel_filtered[i];
        gyro_filtered[i]  = alpha * gyro[i]  + (1.0f - alpha) * gyro_filtered[i];
    }

    // printf("filt_accel:%.6f,%.6f,%.6f, filt_gyro:%.6f,%.6f,%.6f\n\r", 
    //    accel_filtered[0], accel_filtered[1], accel_filtered[2], 
    //    gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);

    // printf("%.6f,%.6f,%.6f\n\r", 
    //    accel_filtered[0], accel_filtered[1], accel_filtered[2]);

    // printf("%.6f,%.6f,%.6f\n\r", 
    //    gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);

    float ax = accel_filtered[0];  // or accel[0]
    float ay = accel_filtered[1];
    float az = accel_filtered[2];

    constexpr float RAD_TO_DEG = 57.2958f;

    float roll  = (atan2(ay, az) * RAD_TO_DEG);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // printf("Pitch: %.2f deg, Roll: %.2f deg\n\r", pitch, roll);

    printf("%.6f,%.6f,%.6f\n\r", 
       roll, pitch, 0.0);
}

int main() {
    // Initialize stdio for debugging (optional)
    stdio_init_all();
    uint32_t loop_start_time;

    sleep_ms(1000);
    printf("Init...\n\r"); 

    Spi_init(SPI_PORT, MISO, MOSI, SCLK, CS);

    LSM6DSOX_init();

    printf("Loop...\n\r"); 

    while (true) {
        loop_start_time = time_us_32();

        LSM6DSOX_read_raw();
        // printf("Before: time: %lu\n\r", time_us_32() - loop_start_time);
        while(time_us_32() - loop_start_time < 1000);
        // printf("After: time: %lu\n\r", time_us_32() - loop_start_time);
    }

    return 0;
}
