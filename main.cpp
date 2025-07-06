#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "lsm6dsox.hpp"
#include "spi.hpp"

#define MISO 16
#define CS 17
#define SCLK 18
#define MOSI 19

#define SPI_PORT spi0

Spi spi_i(SPI_PORT, MISO, MOSI, SCLK, CS);
LSM6DSOX LSM6DSOX_i(&spi_i);

int main() {
    // Initialize stdio for debugging (optional)
    stdio_init_all();
    uint32_t loop_start_time;

    sleep_ms(1000);
    printf("Init...\n\r"); 

    printf("miso: %d, mosi: %d, sclk: %d, cs: %d\n\r", spi_i.miso, spi_i.mosi, spi_i.sclk, spi_i.cs);
    LSM6DSOX_i.LSM6DSOX_init();
    while (true) {
        loop_start_time = time_us_32();

        LSM6DSOX_i.LSM6DSOX_read_raw();

        // printf("Before: time: %lu\n\r", time_us_32() - loop_start_time);
        while(time_us_32() - loop_start_time < 1000);
        // printf("After: time: %lu\n\r", time_us_32() - loop_start_time);
    }

    return 0;
}


/*
rotate around the y axis : azimuth “psi”
rotate around the y axis : pitch “theta”
rotate around the x axis :roll “phi”

p : roll
q : pitch
r : psi
*/