#include <stdio.h>           // Standard I/O library for printing to terminal
#include "pico/stdlib.h"     // Raspberry Pi Pico SDK for GPIO, sleep, etc.
#include "hardware/spi.h"    // SPI interface for communication with the sensor
#include "math.h"            // Math functions (like atan2, sqrt, etc.)

#define MISO 16                 // SPI MISO pin number
#define CS 17                   // SPI Chip Select pin number
#define SCLK 18                 // SPI Clock pin number
#define MOSI 19                 // SPI MOSI pin number
#define SPI_PORT spi0           // SPI hardware instance used
#define SPI_CLK_FREQ 10000000   // SPI SCLK frequnecy

// LSM6DSOX Register Definitions
// Identification register (should return 0x6C for LSM6DSOX)
#define WHO_AM_I         0x0F

// Control Registers
#define CTRL1_XL         0x10  // Accelerometer control (ODR, full-scale, bandwidth)
#define CTRL2_G          0x11  // Gyroscope control (ODR, full-scale)
#define CTRL3_C          0x12  // Common control (BDU, IF_INC, reset, etc.)

// Gyroscope Output Registers (Low and High bytes for each axis)
#define OUTX_L_G         0x22  // Gyro X-axis low byte
#define OUTX_H_G         0x23  // Gyro X-axis high byte
#define OUTY_L_G         0x24  // Gyro Y-axis low byte
#define OUTY_H_G         0x25  // Gyro Y-axis high byte
#define OUTZ_L_G         0x26  // Gyro Z-axis low byte
#define OUTZ_H_G         0x27  // Gyro Z-axis high byte

// Accelerometer Output Registers (Low and High bytes for each axis)
#define OUTX_L_A         0x28  // Accel X-axis low byte
#define OUTX_H_A         0x29  // Accel X-axis high byte
#define OUTY_L_A         0x2A  // Accel Y-axis low byte
#define OUTY_H_A         0x2B  // Accel Y-axis high byte
#define OUTZ_L_A         0x2C  // Accel Z-axis low byte
#define OUTZ_H_A         0x2D  // Accel Z-axis high byte

// Constants
#define CAL_SAMPLES 2000 // Number of samples to collect during calibration
#define ACCEL_SENSITIVITY 0.244f  // Accelerometer sensitivity for ±8g full scale (datasheet value: 0.244 mg/LSB)
#define GYRO_SENSITIVITY 35.0f    // Gyroscope sensitivity for ±1000 dps full scale (datasheet value: 35 mdps/LSB)

// Function Prototypes
void Spi_init(spi_inst_t *spi_inst, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs); // Initialize SPI communication with specified pins and SPI peripheral
void reg_write(spi_inst_t *spi, const uint cs, const uint8_t reg, const uint8_t data); // Write one byte to a register over SPI
int reg_read(spi_inst_t *spi, const uint cs, const uint8_t reg, uint8_t *buf, const uint8_t nbytes); // Read one or more bytes from a register over SPI
int LSM6DSOX_init(); // Initialize the LSM6DSOX IMU (configure control registers, verify WHO_AM_I)
void LSM6DSOX_calibrate(); // Calibrate the IMU by averaging readings over many samples
void LSM6DSOX_read_raw(); // Read raw accelerometer and gyroscope data from output registers
void LSM6DSOX_read(); // Convert raw IMU data to physical units (g, dps)
void LSM6DSOX_read_calibrted(); // Read IMU data and apply calibration offsets


int success;  // Flag indicating whether IMU initialization was successful (0 = success, non-zero = error)
uint8_t buf_raw[12];        // Buffer to store 12 bytes of raw SPI data (6 for gyro, 6 for accel)
int16_t accel_raw[3];       // Raw accelerometer readings: [X, Y, Z], in sensor LSB units
int16_t gyro_raw[3];        // Raw gyroscope readings: [X, Y, Z], in sensor LSB units
float accel[3];             // Accelerometer values converted to physical units [g]
float gyro[3];              // Gyroscope values converted to physical units [dps]
float accel_offset[3];      // Offset (bias) values for accelerometer, determined during calibration [g]
float gyro_offset[3];       // Offset (bias) values for gyroscope, determined during calibration [dps]
float accel_cal[3];         // Final calibrated accelerometer values: accel[] - accel_offset[]
float gyro_cal[3];          // Final calibrated gyroscope values: gyro[] - gyro_offset[]


// Initializes the SPI interface with the given pins and settings
void Spi_init(spi_inst_t *spi_inst, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs) {
    printf("Spi init...\n\r");  // Print debug message

    // Initialize the SPI peripheral with a defined clock frequency
    spi_init(spi_inst, SPI_CLK_FREQ);  // Example: 10 MHz SPI clock

    // Set the GPIO pins to SPI function (hardware-controlled SPI)
    gpio_set_function(miso, GPIO_FUNC_SPI);  // Set MISO pin
    gpio_set_function(mosi, GPIO_FUNC_SPI);  // Set MOSI pin
    gpio_set_function(sclk, GPIO_FUNC_SPI);  // Set SCLK pin

    // Configure SPI data format
    // - 8 bits per transfer
    // - CPOL = 1 (idle clock is high)
    // - CPHA = 1 (data sampled on second clock edge)
    // - MSB sent first
    spi_set_format(spi_inst, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // Initialize and configure the CS (chip select) pin
    gpio_init(cs);           // Initialize chip select GPIO
    gpio_set_dir(cs, GPIO_OUT);  // Set it as output
    gpio_put(cs, 1);         // Set CS high (not selected by default)

    // Print configured pin numbers for confirmation
    printf("Spi_init: miso: %d, mosi: %d, sclk: %d, cs: %d\n\r", MISO, MOSI, SCLK, CS);
}


// Write 1 byte to a specified register on the SPI device
void reg_write(spi_inst_t *spi, const uint cs, const uint8_t reg, const uint8_t data) {
    uint8_t msg[2];

    // Construct the SPI message:
    // - msg[0]: Register address with write bit (bit 7 = 0)
    //   LSM6DSOX expects bit 7 cleared for write operations.
    // - msg[1]: Data byte to write
    msg[0] = 0x7F & reg;  // Clear bit 7 to indicate a write
    msg[1] = data;        // Data to write to the register

    // Begin SPI transaction
    gpio_put(cs, 0);  // Pull chip select low to enable SPI device

    // Send the 2-byte message: [register address, data]
    spi_write_blocking(spi, msg, 2);

    // End SPI transaction
    gpio_put(cs, 1);  // Set chip select high to end communication
}


// Read byte(s) from the specified register over SPI
// If nbytes > 1, reads from consecutive registers (burst read)
int reg_read(spi_inst_t *spi, const uint cs, uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    int num_bytes_read = 0;

    // Return error if invalid number of bytes requested
    if (nbytes < 1) {
        return -1;
    }

    // Set the read bit (bit 7 = 1) to indicate a read operation
    // LSM6DSOX uses MSB = 1 for read, MSB = 0 for write
    reg = 0x80 | reg;

    // Begin SPI transaction
    gpio_put(cs, 0);  // Pull chip select low to select the device

    // Send register address with read bit set
    spi_write_blocking(spi, &reg, 1);

    // Read nbytes into buffer (sending dummy 0s while receiving)
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);

    // End SPI transaction
    gpio_put(cs, 1);  // Set chip select high to end communication

    // Return the number of bytes read
    return num_bytes_read;
}



int LSM6DSOX_init() {
    uint8_t buf[1];
    int num_bytes_read = 0;

    printf("LSM6DSOX_init...\n\r");  // Print a debug message to indicate initialization has started

    // WHO_AM_I check
    num_bytes_read = reg_read(SPI_PORT, CS, WHO_AM_I, buf, 1);
    if (buf[0] != 0x6C) {
        printf("WHO_AM_I mismatch: got 0x%02X\n\r", buf[0]);
        return 1;
    }
    printf("WHO_AM_I : 0x%02X\n\r", buf[0]);

    // CTRL1_XL: Accel @ 1.66 kHz, ±8g, page 54 
    uint8_t LPF2_XL_EN = 0b00000000; // Accelerometer high-resolution selection (default = 0)
    uint8_t FS_A = 0b00001100; // Accelerometer full-scale selection, ±8g
    uint8_t ODR_XL_A = 0b10000000; // Accelerometer ODR selection, 1.66 kHz
    reg_write(SPI_PORT, CS, CTRL1_XL, ODR_XL_A | FS_A | LPF2_XL_EN);
    reg_read(SPI_PORT, CS, CTRL1_XL, buf, 1);
    if (buf[0] != (ODR_XL_A | FS_A | LPF2_XL_EN)) {
        printf("CTRL1_XL mismatch: got 0x%02X\n\r", buf[0]);
        return 1;
    }
    printf("CTRL1_XL : 0x%02X\n\r", buf[0]);

    // CTRL2_G: Gyro @ 833 Hz, ±1000 dps, page 55
    uint8_t FS_125 = 0b00000000; // Selects gyroscope UI chain full-scale ±125 dp (default = 0)
    uint8_t FS_G = 0b00001000; // Gyroscope UI chain full-scale selection, ±1000 dps
    uint8_t ODR_XL_G = 0b10000000; // Gyroscope output data rate selection, 1.66 kHz
    reg_write(SPI_PORT, CS, CTRL2_G, ODR_XL_G | FS_G | FS_125);
    reg_read(SPI_PORT, CS, CTRL2_G, buf, 1);
    if (buf[0] != (ODR_XL_G | FS_G | FS_125)) {
        printf("CTRL2_G mismatch: got 0x%02X\n\r", buf[0]);
        return 1;
    }
    printf("CTRL2_G : 0x%02X\n\r", buf[0]);

    // CTRL3_C: IF_INC=1 (auto-increment), BDU=1 (block data update), page 56
    uint8_t BDU = 0b01000000; // Block data update: output registers are not updated until MSB and LSB have been read
    uint8_t IF_INC = 0b00000100; // Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI): enabled 
    reg_write(SPI_PORT, CS, CTRL3_C, BDU | IF_INC);
    reg_read(SPI_PORT, CS, CTRL3_C, buf, 1);
    if (buf[0] != (BDU | IF_INC)) {
        printf("CTRL3_C mismatch: got 0x%02X\n\r", buf[0]);
        return 1;
    }
    printf("CTRL3_C : 0x%02X\n\r", buf[0]);

    return 0;
}

// Function to calibrate the LSM6DSOX accelerometer and gyroscope
void LSM6DSOX_calibrate() {

    // Print a message to indicate calibration has started
    printf("Calibrating...\n\r");

    // Initialize offsets to zero
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = 0;  // Reset accelerometer offset for X, Y, Z
        gyro_offset[i] = 0;   // Reset gyroscope offset for X, Y, Z
    }

    // Take multiple samples and accumulate the readings
    for (int i = 0; i < CAL_SAMPLES; i++) {
        LSM6DSOX_read();  // Read uncalibrated sensor data into accel[] and gyro[]
        for (int j = 0; j < 3; j++) {
            accel_offset[j] += accel[j];  // UncalibratedAccumulate accelerometer readings
            gyro_offset[j] += gyro[j];    // Uncalibrated Accumulate gyroscope readings
        }
        sleep_ms(1);  // Small delay between samples (1 millisecond)
    }

    // Average the accumulated readings to calculate offsets
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = accel_offset[i] / (float)(CAL_SAMPLES);  // Average accel
        gyro_offset[i]  = gyro_offset[i] / (float)(CAL_SAMPLES);   // Average gyro
        printf("Average [%d]: accel ave: %f, gyro ave: %f\n\r", i, accel_offset[i], gyro_offset[i]);
    }

    // Adjust Z-axis acceleration offset to account for gravity (1g)
    // Assumes the sensor is lying flat during calibration, so Z should read 1g
    accel_offset[2] = accel_offset[2] - 1;

    // Step 5: Print final offsets
    for (int i = 0; i < 3; i++) {
        printf("Offset [%d]: accel_offset: %f, gyro_offset: %f\n\r", i, accel_offset[i], gyro_offset[i]);
    }
}


// Reads 12 bytes of raw data from the LSM6DSOX IMU: 6 bytes for gyroscope and 6 for accelerometer
void LSM6DSOX_read_raw() {
    // Read 12 bytes starting from OUTX_L_G (gyro X low byte register)
    int num_bytes_read = reg_read(SPI_PORT, CS, OUTX_L_G, buf_raw, 12);

    // Check if the correct number of bytes were read
    if (num_bytes_read != 12) {
        printf("Error: LSM6DSOX_read_raw failed to read 12 bytes\n\r");
        return; // Exit early if read failed
    }
    
    // Optional debug: print raw byte values in hex
    // for(int i = 0; i < 12; i++) {
    //     printf("%x, ", buf_raw[i]);
    //     if (i == 11) {
    //         printf("%x\n\r", buf_raw[i]);
    //     }
    // }

    // === Extract gyroscope raw data (X, Y, Z) ===
    // Combine low and high bytes to form signed 16-bit integers
    gyro_raw[0] = (int16_t)(buf_raw[1] << 8 | buf_raw[0]);  // Gyro X
    gyro_raw[1] = (int16_t)(buf_raw[3] << 8 | buf_raw[2]);  // Gyro Y
    gyro_raw[2] = (int16_t)(buf_raw[5] << 8 | buf_raw[4]);  // Gyro Z

    // === Extract accelerometer raw data (X, Y, Z) ===
    // Combine low and high bytes to form signed 16-bit integers
    accel_raw[0] = (int16_t)(buf_raw[7]  << 8 | buf_raw[6]);  // Accel X
    accel_raw[1] = (int16_t)(buf_raw[9]  << 8 | buf_raw[8]);  // Accel Y
    accel_raw[2] = (int16_t)(buf_raw[11] << 8 | buf_raw[10]); // Accel Z

    // Optional debug: print raw values as integers
    // printf("gyro_raw:%d,%d,%d, accel_raw:%d,%d,%d\n\r",
    //        gyro_raw[0], gyro_raw[1], gyro_raw[2],
    //        accel_raw[0], accel_raw[1], accel_raw[2]);

}


// Reads raw IMU data and converts it into physical units: acceleration [g] and angular velocity [dps]
void LSM6DSOX_read() {

    LSM6DSOX_read_raw();  // Read raw data from IMU into accel_raw[] and gyro_raw[]

    // Convert accelerometer raw data to physical units (g-forces), 1 g-force = 9.8 m/s^2
    // ACCEL_SENSITIVITY is in [mg/LSB], so divide by 1000 to get [g]
    accel[0] = (accel_raw[0] * ACCEL_SENSITIVITY) / 1000.0f;  // Accel X
    accel[1] = (accel_raw[1] * ACCEL_SENSITIVITY) / 1000.0f;  // Accel Y
    accel[2] = (accel_raw[2] * ACCEL_SENSITIVITY) / 1000.0f;  // Accel Z

    // Convert gyroscope raw data to physical units (degrees per second)
    // GYRO_SENSITIVITY is in [mdps/LSB], so divide by 1000 to get [dps]
    gyro[0] = (gyro_raw[0] * GYRO_SENSITIVITY) / 1000.0f;  // Gyro X
    gyro[1] = (gyro_raw[1] * GYRO_SENSITIVITY) / 1000.0f;  // Gyro Y
    gyro[2] = (gyro_raw[2] * GYRO_SENSITIVITY) / 1000.0f;  // Gyro Z

    
    // Optional: Print converted data for debugging
    // printf("accel:%.6f,%.6f,%.6f, gyro:%.6f,%.6f,%.6f\n\r",
    //        accel[0], accel[1], accel[2],
    //        gyro[0], gyro[1], gyro[2]);
    
}


// Reads IMU data, converts it to physical units, and applies calibration offsets
void LSM6DSOX_read_calibrted() {

    LSM6DSOX_read();  // Read raw IMU data and convert to physical units (g and dps)

    // Subtract accelerometer offsets to get calibrated acceleration [g]
    accel_cal[0] = accel[0] - accel_offset[0];  // Calibrated Accel X
    accel_cal[1] = accel[1] - accel_offset[1];  // Calibrated Accel Y
    accel_cal[2] = accel[2] - accel_offset[2];  // Calibrated Accel Z

    // Subtract gyroscope offsets to get calibrated angular velocity [dps]
    gyro_cal[0] = gyro[0] - gyro_offset[0];    // Calibrated Gyro X
    gyro_cal[1] = gyro[1] - gyro_offset[1];    // Calibrated Gyro Y
    gyro_cal[2] = gyro[2] - gyro_offset[2];    // Calibrated Gyro Z

    // Print calibrated values for debugging/monitoring
    printf("accel_cal:\t%.6f, \t%.6f, \t%.6f,\t gyro_cal:\t%.6f,\t%.6f,\t%.6f\t\n\r",
           accel_cal[0], accel_cal[1], accel_cal[2],
           gyro_cal[0], gyro_cal[1], gyro_cal[2]);
}


int main() {
    stdio_init_all();  // Initialize all standard I/O (usually for USB UART on the Pico, enabling printf)

    sleep_ms(1000);    // Wait for 1 second after startup (allow USB host to connect, IMU to stabilize)

    Spi_init(SPI_PORT, MISO, MOSI, SCLK, CS); // Initialize SPI communication (set SPI peripheral, pins, and config)

    success = LSM6DSOX_init();  // Initialize the LSM6DSOX IMU sensor (configure registers, check connection)

    LSM6DSOX_calibrate();  // Calibrate the IMU (compute and store gyro/accel offsets)

    // Check if the IMU initialization was successful
    if (success != 0) {
        printf("Something went wrong.\n\r");  // Print error if IMU init failed
    } else {
        while (true) {  // Infinite loop — typical pattern for embedded main loop

            // Uncomment if you want to read raw or converted data:
            // LSM6DSOX_read_raw();     // Read raw accel and gyro values (no filtering or offset correction)
            // LSM6DSOX_read();         // Read and apply conversion

            LSM6DSOX_read_calibrted();  // Read sensor data and apply calibration offsets

            sleep_ms(100);  // Delay for 100 ms between samples (~10 Hz read rate)
        }
    }

    return 0;  // This line is never reached due to infinite loop
}


