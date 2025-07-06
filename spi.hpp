#ifndef SPI_HPP
#define SPI_HPP

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "math.h"


class Spi {
    public:
        spi_inst_t *spi_inst;
        uint8_t miso, mosi, sclk, cs;

        Spi(spi_inst_t *spi, 
                        uint8_t miso, 
                        uint8_t mosi, 
                        uint8_t sclk, 
                        uint8_t cs);

        static void reg_write( spi_inst_t *spi, 
                        const uint cs, 
                        const uint8_t reg, 
                        const uint8_t data);

        static int reg_read(  spi_inst_t *spi,
                        const uint cs,
                        const uint8_t reg,
                        uint8_t *buf,
                        uint8_t nbytes);

    private:

};
#endif