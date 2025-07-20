#include <stdio.h>           // Standard I/O library for printing to terminal
#include <string.h>          // String manipulation functions
#include "pico/stdlib.h"     // Raspberry Pi Pico SDK for GPIO, sleep, etc.
#include "hardware/spi.h"    // SPI interface for communication with the sensor
#include "math.h"            // Math functions (like atan2, sqrt, etc.)

#define MISO 16              // SPI MISO pin number
#define CS 17                // SPI Chip Select pin number
#define SCLK 18              // SPI Clock pin number
#define MOSI 19              // SPI MOSI pin number
#define SPI_PORT spi0        // SPI hardware instance used

// Identification
#define WHO_AM_I         0x0F

// Control Registers
#define CTRL1_XL         0x10  // Accelerometer control
#define CTRL2_G          0x11  // Gyroscope control
#define CTRL3_C          0x12  // Common control (BDU, IF_INC, etc.)
#define CTRL4_C          0x13
#define CTRL5_C          0x14
#define CTRL6_C          0x15  // Gyro LPF
#define CTRL7_G          0x16  // Gyro settings
#define CTRL8_XL         0x17  // Accel filter settings
#define CTRL9_XL         0x18
#define CTRL10_C         0x19

// Status & Interrupts
#define STATUS_REG       0x1E
#define INT1_CTRL        0x0D
#define INT2_CTRL        0x0E

// Output Registers (Gyroscope)
#define OUTX_L_G         0x22  // 0010 = 2, 00100010, write 1010_0010
#define OUTX_H_G         0x23
#define OUTY_L_G         0x24
#define OUTY_H_G         0x25
#define OUTZ_L_G         0x26
#define OUTZ_H_G         0x27

// Output Registers (Accelerometer)
#define OUTX_L_A         0x28
#define OUTX_H_A         0x29
#define OUTY_L_A         0x2A
#define OUTY_H_A         0x2B
#define OUTZ_L_A         0x2C
#define OUTZ_H_A         0x2D

// FIFO (optional)
#define FIFO_CTRL1       0x07
#define FIFO_CTRL2       0x08
#define FIFO_CTRL3       0x09
#define FIFO_CTRL4       0x0A
#define FIFO_STATUS1     0x3A
#define FIFO_STATUS2     0x3B
#define FIFO_DATA_OUT_L  0x3E
#define FIFO_DATA_OUT_H  0x3F

#define CAL_SAMPLES 5000

int LSM6DSOX_init();
void LSM6DSOX_read_raw();
void LSM6DSOX_read();

uint8_t buf_raw[12];                     // Buffer for raw SPI read
int16_t accel_raw[3];                    // Raw accelerometer values (X, Y, Z)
int16_t gyro_raw[3];                     // Raw gyroscope values (X, Y, Z)
float accel[3];                          // Scaled accelerometer values [g]
float gyro[3];                           // Scaled gyroscope values [dps]
float accel_offset[3];                   // Scaled accelerometer values [g]
float gyro_offset[3];                    // Scaled gyroscope values [dps]
float accel_cal[3];                  // Scaled accelerometer values [g]
float gyro_cal[3];                   // Scaled gyroscope values [dps]
float accel_filtered[3];                 // Low-pass filtered accel
float gyro_filtered[3];                  // Low-pass filtered gyro

void Spi_init(spi_inst_t *spi_inst, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs) {
    printf("Spi init...\n\r");
    spi_init(spi_inst, 10000000);  // 10 MHz (super stable)
    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);

    spi_set_format (spi_inst, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

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

    if (nbytes < 1) {
        return -1;
    }
    
    // Construct message (set ~W bit high)
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

    return 0;
}

void LSM6DSOX_calibrate() {

    printf("Calibrating...");

    for (int i = 0; i < 3; i++) {
        accel_offset[i] = 0;
        gyro_offset[i] = 0;
    }

    for (int i = 0; i < CAL_SAMPLES; i++) {
        LSM6DSOX_read();
        for (int j = 0; j < 3; j++) {
            accel_offset[j] += accel[j];
            gyro_offset[j] += gyro[j];
            // printf("Summing: accel_offset[%d]: %f, gyro_offset[%d]: %f\n\r", j, accel_offset[j], j, gyro_offset[j]);
        }
        sleep_ms(1);
    }
    
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = accel_offset[i]/(float)(CAL_SAMPLES);
        gyro_offset[i] = gyro_offset[i]/(float)(CAL_SAMPLES);
        printf("Average: accel_offset[%d]: %f, gyro_offset[%d]: %f\n\r", i, accel_offset[i], i, gyro_offset[i]);
    }

    accel_offset[2] = accel_offset[2] - 1;

    for (int i = 0; i < 3; i++) {
        printf("Average: accel_offset[%d]: %f, gyro_offset[%d]: %f\n\r", i, accel_offset[i], i, gyro_offset[i]);
    }
}

void LSM6DSOX_read_raw() {
    int num_bytes_read = reg_read(SPI_PORT, CS, OUTX_L_G, buf_raw, 12);
    if (num_bytes_read != 12) {
        printf("Error: LSM6DSOX_read_raw failed to read 12 bytes\n\r");
        return;
    }

    // for(int i =0; i < 12; i++) {
    //     printf("%x, ", buf_raw[i]);
    //     if(i == 11) {
    //         printf("%x\n\r", buf_raw[i]);
    //     }
    // }

    // Raw gyroscope data
    gyro_raw[0] = (int16_t)(buf_raw[1] << 8 | buf_raw[0]);
    gyro_raw[1] = (int16_t)(buf_raw[3] << 8 | buf_raw[2]);
    gyro_raw[2] = (int16_t)(buf_raw[5] << 8 | buf_raw[4]);

    // Raw accelerometer data
    accel_raw[0] = (int16_t)(buf_raw[7]  << 8 | buf_raw[6]);
    accel_raw[1] = (int16_t)(buf_raw[9]  << 8 | buf_raw[8]);
    accel_raw[2] = (int16_t)(buf_raw[11] << 8 | buf_raw[10]);

    // printf("gyro_raw:%d,%d,%d, accel_raw:%d,%d,%d\n\r", gyro_raw[0], gyro_raw[1], gyro_raw[2], accel_raw[0], accel_raw[1], accel_raw[2]);
}

void LSM6DSOX_read() {
    constexpr float ACCEL_SENSITIVITY = 0.244f;   // [mg/LSB] for ±8g
    constexpr float GYRO_SENSITIVITY  = 35.0f;     // [mdps/LSB] for ±1000 dps

    LSM6DSOX_read_raw();

    // Convert to physical units: [g] and [dps]
    accel[0] = (accel_raw[0] * ACCEL_SENSITIVITY) / 1000.0f;
    accel[1] = (accel_raw[1] * ACCEL_SENSITIVITY) / 1000.0f;
    accel[2] = (accel_raw[2] * ACCEL_SENSITIVITY) / 1000.0f;

    gyro[0] = (gyro_raw[0] * GYRO_SENSITIVITY) / 1000.0f;
    gyro[1] = (gyro_raw[1] * GYRO_SENSITIVITY) / 1000.0f;
    gyro[2] = (gyro_raw[2] * GYRO_SENSITIVITY) / 1000.0f;

    // printf("accel:%.6f,%.6f,%.6f, gyro:%.6f,%.6f,%.6f\n\r", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
}

void LSM6DSOX_read_calibrted() {

    LSM6DSOX_read();

    // Convert to physical units: [g] and [dps]
    accel_cal[0] = accel[0] - accel_offset[0];
    accel_cal[1] = accel[1] - accel_offset[1];
    accel_cal[2] = accel[2] - accel_offset[2];

    gyro_cal[0] = gyro[0] - gyro_offset[0];
    gyro_cal[1] = gyro[1] - gyro_offset[1];
    gyro_cal[2] = gyro[2] - gyro_offset[2];

    // printf("accel_cal:%.6f, \t%.6f, \t%.6f,\t gyro_cal:%.6f,\t%.6f,\t%.6f\t\n\r", accel_cal[0], accel_cal[1], accel_cal[2], gyro_cal[0], gyro_cal[1], gyro_cal[2]);

    constexpr float alpha = 0.1f;  // smoothing factor: smaller = smoother

    for (int i = 0; i < 3; ++i) {
        accel_filtered[i] = alpha * accel_cal[i] + (1.0f - alpha) * accel_filtered[i];
        gyro_filtered[i]  = alpha * gyro_cal[i]  + (1.0f - alpha) * gyro_filtered[i];
    }

    // printf("filt_accel:%.6f,%.6f,%.6f, filt_gyro:%.6f,%.6f,%.6f\n\r", 
    //    accel_filtered[0], accel_filtered[1], accel_filtered[2], 
    //    gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);

    float ax = accel_filtered[0];  // or accel[0]
    float ay = accel_filtered[1];
    float az = accel_filtered[2];

    constexpr float RAD_TO_DEG = 57.2958f;

    float roll  = (atan2(ay, az) * RAD_TO_DEG);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    printf("Pitch: %.2f deg, \t\t Roll: %.2f deg\n\r", pitch, roll);
}

void LSM6DSOX_read_filtered() {

    LSM6DSOX_read_calibrted();

    constexpr float alpha = 0.1f;  // smoothing factor: smaller = smoother

    for (int i = 0; i < 3; ++i) {
        accel_filtered[i] = alpha * accel_cal[i] + (1.0f - alpha) * accel_filtered[i];
        gyro_filtered[i]  = alpha * gyro_cal[i]  + (1.0f - alpha) * gyro_filtered[i];
    }

    // printf("filt_accel:%.6f,%.6f,%.6f, filt_gyro:%.6f,%.6f,%.6f\n\r", 
    //    accel_filtered[0], accel_filtered[1], accel_filtered[2], 
    //    gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);

    float ax = accel_filtered[0];  // or accel[0]
    float ay = accel_filtered[1];
    float az = accel_filtered[2];

    constexpr float RAD_TO_DEG = 57.2958f;

    float roll  = (atan2(ay, az) * RAD_TO_DEG);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    printf("Pitch: %.2f deg, \t\t Roll: %.2f deg\n\r", pitch, roll);
}

void LSM6DSOX_read_angle() {

    LSM6DSOX_read_filtered();

    float ax = accel_filtered[0];  // or accel[0]
    float ay = accel_filtered[1];
    float az = accel_filtered[2];

    constexpr float RAD_TO_DEG = 57.2958f;

    float roll  = (atan2(ay, az) * RAD_TO_DEG);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    printf("Pitch: %.2f deg, \t\t Roll: %.2f deg\n\r", pitch, roll);
}

int main() {
    stdio_init_all();                   // Initialize USB stdio (for printf)

    sleep_ms(1000);                    // Delay 1 second after power-up

    printf("Init...\n\r");             // Debug print
    Spi_init(SPI_PORT, MISO, MOSI, SCLK, CS); // Setup SPI pins and clock
    LSM6DSOX_init();                          // Initialize IMU sensor

    LSM6DSOX_calibrate();

    uint32_t loop_start_time;          // Timestamp for timing control
    while (true) {
        loop_start_time = time_us_32();       // Get current microsecond time
        // LSM6DSOX_read_raw();                  // Read and process sensor data
        LSM6DSOX_read_calibrted();
        // printf("Before: %d\n\r", time_us_32() - loop_start_time);             // Debug print
        while(time_us_32() - loop_start_time < 1000); // Wait to maintain 1 kHz loop
        // printf("After: %d\n\r", time_us_32() - loop_start_time); 
    }

    return 0; // This will never be reached
}

