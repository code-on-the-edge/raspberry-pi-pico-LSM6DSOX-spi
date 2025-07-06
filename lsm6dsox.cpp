#include "lsm6dsox.hpp"

LSM6DSOX::LSM6DSOX(Spi *spi_ptr_in) {
    spi_ptr = spi_ptr_in;
}

int LSM6DSOX::LSM6DSOX_init() {
    uint8_t buf[1];
    int num_bytes_read = 0;

    printf("LSM6DSOX_init: miso: %d, mosi: %d, sclk: %d, cs: %d\n\r",
           spi_ptr->miso, spi_ptr->mosi, spi_ptr->sclk, spi_ptr->cs);

    // WHO_AM_I check
    num_bytes_read = Spi::reg_read(spi_ptr->spi_inst, spi_ptr->cs, WHO_AM_I, buf, 1);
    if (buf[0] != 0x6C) {
        printf("WHO_AM_I mismatch: got 0x%02X\n\r", buf[0]);
        return -1;
    }
    printf("WHO_AM_I : 0x%02X\n\r", buf[0]);

    // CTRL3_C: IF_INC=1 (auto-increment), BDU=1 (block data update)
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL3_C, 0b01000100);
    Spi::reg_read(spi_ptr->spi_inst, spi_ptr->cs, CTRL3_C, buf, 1);
    printf("CTRL3_C : 0x%02X\n\r", buf[0]);

    // CTRL1_XL: Accel @ 833 Hz, ±8g
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL1_XL, 0b10001100);
    Spi::reg_read(spi_ptr->spi_inst, spi_ptr->cs, CTRL1_XL, buf, 1);
    printf("CTRL1_XL : 0x%02X\n\r", buf[0]);

    // CTRL2_G: Gyro @ 833 Hz, ±1000 dps
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL2_G, 0b10001000);
    Spi::reg_read(spi_ptr->spi_inst, spi_ptr->cs, CTRL2_G, buf, 1);
    printf("CTRL2_G : 0x%02X\n\r", buf[0]);

    // CTRL6_C: Gyro LPF1 @ ODR/20
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL6_C, 0b00000010);

    // CTRL8_XL: Accel LPF2 @ ODR/20
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL8_XL, 0b00000010);

    // CTRL7_G: High-performance gyro
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL7_G, 0b00000000);

    // CTRL5_C: Normal mode
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, CTRL5_C, 0b00000000);

    // INT1_CTRL: DRDY accel + gyro on INT1
    Spi::reg_write(spi_ptr->spi_inst, spi_ptr->cs, INT1_CTRL, 0b00000011);

    return 0;
}


void LSM6DSOX::LSM6DSOX_read_raw() {
    constexpr float ACCEL_SENSITIVITY = 0.244f;   // [mg/LSB] for ±8g
    constexpr float GYRO_SENSITIVITY  = 35.0f;     // [mdps/LSB] for ±1000 dps

    int num_bytes_read = Spi::reg_read(spi_ptr->spi_inst, spi_ptr->cs, OUTX_L_G, buf_raw, 12);
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

