#include <xspi.h>

int sx127x_read_reg(struct spi_dt_spec *spi, uint8_t reg, uint8_t *val);
int sx127x_write_reg(struct spi_dt_spec *spi, uint8_t reg, uint8_t val);
