#include <sx127x.h>

int sx127x_read_reg(struct spi_dt_spec *spi, uint8_t reg, uint8_t *val)
{
    return spi_write_read(spi, &reg, 1, &val, 1);
}

int sx127x_write_reg(struct spi_dt_spec *spi, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg | 0x80, val};
    return spi_write(spi, buf, 2);
}
