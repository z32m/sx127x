#ifndef __sx127x_h__
#define __sx127x_h__

#include <xspi.h>

#include <sx1276Regs-Fsk.h>
#include <sx1276Regs-LoRa.h>
#include <bits.h>

typedef enum
{
    SF_6 = RFLR_MODEMCONFIG2_SF_6,
    SF_7 = RFLR_MODEMCONFIG2_SF_7,
    SF_8 = RFLR_MODEMCONFIG2_SF_8,
    SF_9 = RFLR_MODEMCONFIG2_SF_9,
    SF_10 = RFLR_MODEMCONFIG2_SF_10,
    SF_11 = RFLR_MODEMCONFIG2_SF_11,
    SF_12 = RFLR_MODEMCONFIG2_SF_12,
} lora_datarate_t;

typedef enum
{
    CR_4_5 = RFLR_MODEMCONFIG1_CODINGRATE_4_5,
    CR_4_6 = RFLR_MODEMCONFIG1_CODINGRATE_4_6,
    CR_4_7 = RFLR_MODEMCONFIG1_CODINGRATE_4_7,
    CR_4_8 = RFLR_MODEMCONFIG1_CODINGRATE_4_8,
} lora_coding_rate_t;

typedef enum
{
    BW_125_KHZ = RFLR_MODEMCONFIG1_BW_125_KHZ,
    BW_250_KHZ = RFLR_MODEMCONFIG1_BW_250_KHZ,
    BW_500_KHZ = RFLR_MODEMCONFIG1_BW_500_KHZ,
} lora_bandwidth_t;

/*
Implicit Header Mode
In certain scenarios, where the payload, coding rate and CRC presence are fixed or known in advance, it may be
advantageous to reduce transmission time by invoking implicit header mode. In this mode the header is removed from the
packet. In this case the payload length, error coding rate and presence of the payload CRC must be manually configured
on both sides of the radio link.
Note With SF = 6 selected, implicit header mode is the only mode of operation possible.
*/
typedef enum
{
    implicit_header_on = RFLR_MODEMCONFIG1_IMPLICITHEADER_ON,
    implicit_header_off = RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF
} implicit_header_t;

typedef struct
{
    uint32_t freq;
    lora_bandwidth_t bandwidth;
    lora_coding_rate_t coding_rate;
    lora_datarate_t data_rate;
    implicit_header_t implicit_header;
    uint16_t preamble_len;
    uint8_t packet_len;
} sx127x_config_t;

DEFINE_BITS(sx127x_reg_opmode);
DEFINE_BITS(sx127x_reg_irqflags);

int sx127x_read_reg(const spi_t *spi, uint8_t reg, uint8_t *val);
int sx127x_write_reg(const spi_t *spi, uint8_t reg, uint8_t val);
int sx127x_set_freq(const spi_t *spi, uint32_t freq);

int sx127x_get_opmode(const spi_t *spi, uint8_t *mode);
int sx127x_update_opmode(const spi_t *spi, uint8_t mode);
int sx127x_update_modem_mode(const spi_t *spi, uint8_t mode);

int sx127x_configure_lora(const spi_t *spi, sx127x_config_t *config);

#endif