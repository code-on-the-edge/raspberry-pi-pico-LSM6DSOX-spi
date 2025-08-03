#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define MISO 16 // RX
#define CSn 17 
#define SCLK 18
#define MOSI 19 // TX
#define SPI_CORE spi0
#define SPI_CLK_FREQ 400000

// LSM6DSOX Register Definitions
// Identification register (should return 0x6C for LSM6DSOX)
#define WHO_AM_I         0x0F

// Control Registers
#define CTRL1_XL         0x10  // Accelerometer control (ODR, full-scale, bandwidth)
#define CTRL2_G          0x11  // Gyroscope control (ODR, full-scale)
#define CTRL3_C          0x12  // Common control (BDU, IF_INC, reset, etc.)

// Gyroscope Output Registers (Low and High bytes for each axis)
#define OUTX_L_G         0x22  // Gyro X-axis low byte

uint8_t imu_raw_buf[12];
int16_t gyro_raw[3];
int16_t accel_raw[3];
float gyro[3];
float accel[3];

void spi_core_init(spi_inst_t *spi_inst, uint8_t miso,  uint8_t mosi, uint8_t sclk, uint8_t cs) {
    spi_init(spi_inst, SPI_CLK_FREQ);

    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);
    gpio_set_function(cs, GPIO_FUNC_SPI);

    spi_set_format(spi_inst, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

}

int reg_read(spi_inst_t *spi_inst, const uint8_t cs, uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    int num_bytes_read = 0;

    if(nbytes < 1) {
        return -1;
    }

    reg = 0x80 | reg; // 0b10000001

    gpio_put(cs, 0);
    spi_write_blocking(spi_inst, &reg, 1);
    num_bytes_read = spi_read_blocking(spi_inst, 0, buf, nbytes);
    gpio_put(cs, 1);

    return num_bytes_read;
}

void reg_write(spi_inst_t *spi_inst, const uint8_t cs, const uint8_t reg, const uint8_t data) {
    uint8_t msg[2];

    msg[0] = 0x7F & reg; // 0b00000001
    msg[1] = data;

    gpio_put(cs, 0);
    spi_write_blocking(spi_inst, msg, 2);
    gpio_put(cs, 1);

}

int imu_init() {
    uint8_t buf[1];
    int num_bytes_read = 0;

    num_bytes_read = reg_read(SPI_CORE, CSn, WHO_AM_I, buf, 1);
    printf("WHO_AM_I : %x\n\r", buf[0]);

    reg_write(SPI_CORE, CSn, CTRL1_XL, 0b10001100);
    num_bytes_read = reg_read(SPI_CORE, CSn, CTRL1_XL, buf, 1);
    printf("CTRL1_XL : %x\n\r", buf[0]);

    reg_write(SPI_CORE, CSn, CTRL2_G, 0b10001000);
    num_bytes_read = reg_read(SPI_CORE, CSn, CTRL2_G, buf, 1);
    printf("CTRL2_G : %x\n\r", buf[0]);
    
    reg_write(SPI_CORE, CSn, CTRL3_C, 0b00000100);
    num_bytes_read = reg_read(SPI_CORE, CSn, CTRL3_C, buf, 1);
    printf("CTRL3_C : %x\n\r", buf[0]);

    return num_bytes_read;
}

void read_imu() {
    int num_bytes_read = 0;
    num_bytes_read = reg_read(SPI_CORE, CSn, OUTX_L_G, imu_raw_buf, 12);

    // for(int i = 0; i < 12; i++) {
    //     printf("%x ", imu_raw_buf[i]);
    //     if(i == 11) {
    //         printf("\n\r");
    //     }
    // }

    gyro_raw[0] = (int16_t)(imu_raw_buf[1] << 8 | imu_raw_buf[0]); //x roll
    gyro_raw[1] = (int16_t)(imu_raw_buf[3] << 8 | imu_raw_buf[2]); // y pitch
    gyro_raw[2] = (int16_t)(imu_raw_buf[5] << 8 | imu_raw_buf[4]); //z yaw

    accel_raw[0] = (int16_t)(imu_raw_buf[7] << 8 | imu_raw_buf[6]);// x
    accel_raw[1] = (int16_t)(imu_raw_buf[9] << 8 | imu_raw_buf[8]); // y
    accel_raw[2] = (int16_t)(imu_raw_buf[11] << 8 | imu_raw_buf[10]); // z


    gyro[0] = gyro_raw[0] * 35.0f / 1000.0f; // [LSB] * [mdps/LSB] = mdps, mdps / 1000 = dps
    gyro[1] = gyro_raw[1] * 35.0f / 1000.0f; // [LSB] * [mdps/LSB] = mdps, mdps / 1000 = dps
    gyro[2] = gyro_raw[2] * 35.0f / 1000.0f; // [LSB] * [mdps/LSB] = mdps, mdps / 1000 = dps

    accel[0] = accel_raw[0] * 0.244f / 1000.0f; // [LSB] * [mg/LSB] = mg, mg / 1000 = g, 1g = 9.8 m/s^2
    accel[1] = accel_raw[1] * 0.244f / 1000.0f; // [LSB] * [mg/LSB] = mg, mg / 1000 = g;
    accel[2] = accel_raw[2] * 0.244f / 1000.0f; // [LSB] * [mg/LSB] = mg, mg / 1000 = g;

    printf("Gx: %0.3f, \tGy: %0.3f, \tGz: %0.3f, \tAx: %0.3f, \tAy: %0.3f, \tAz: %0.3f \n\r", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

int main() {
    stdio_init_all();

    spi_core_init(SPI_CORE, MISO, MOSI, SCLK, CSn);

    imu_init();
    
    while(1) {
        
        read_imu();
        
        sleep_ms(1000);
    }
    return 0;
}