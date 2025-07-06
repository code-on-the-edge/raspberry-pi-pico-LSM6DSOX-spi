#include "spi.hpp"

Spi::Spi(spi_inst_t *spi_inst, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs)
    : spi_inst(spi_inst), miso(miso), mosi(mosi), sclk(sclk), cs(cs) {
    printf("Spi init...\n\r");
    spi_init(spi_inst, 4000000);  // 4 MHz (super stable)
    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);

    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

    printf("miso: %d, mosi: %d, sclk: %d, cs: %d\n\r", this->miso, this->mosi, this->sclk, this->cs);
}


// Write 1 byte to the specified register
void Spi::reg_write( spi_inst_t *spi, 
                const uint cs, 
                const uint8_t reg, 
                const uint8_t data) {

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
int Spi::reg_read(  spi_inst_t *spi,
                const uint cs,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Determine if multiple byte (MB) bit should be set
    if (nbytes < 1) {
        return -1;
    } else if (nbytes == 1) {
        mb = 0;
    } else {
        mb = 1;
    }

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