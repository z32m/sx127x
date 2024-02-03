#ifndef __sx127x_h__
#define __sx127x_h__

#include <xspi.h>

#include <sx1276Regs-Fsk.h>
#include <sx1276Regs-LoRa.h>
#include <bits.h>
#include <xglob.h>
#include <xgpio.h>

//  SF7 to SF9 at 125kHz,
//  SF7 to SF10 at 250kHz,
//  SF7 to SF11 at 500kHz
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
} sx127x_modem_config_t;

DEFINE_BITS(sx127x_reg_opmode);
DEFINE_BITS(sx127x_reg_irqflags);

typedef struct
{
    spi_dt_spec_t spi;
    gpio_dt_spec_t reset;
    gpio_dt_spec_t dio0;
} sx127x_dt_spec_t;

typedef enum
{
    iqf_timeout = 1 << 7,
    iqf_rx_done = 1 << 6,
    iqf_payload_crc_error = 1 << 5,
    iqf_valid_header = 1 << 4,
    iqf_tx_done = 1 << 3,
    iqf_cad_done = 1 << 2,
    iqf_tifhss_changed_channelmeout = 1 << 1,
    iqf_cad_detected = 1 << 0,
} sx127x_irq_flags_t;

typedef enum
{
    SX_SLEEP = RF_OPMODE_SLEEP,
    SX_STANDBY = RF_OPMODE_STANDBY,
    SX_STX = RF_OPMODE_SYNTHESIZER_TX,
    SX_TX = RF_OPMODE_TRANSMITTER,
    SX_SRX = RF_OPMODE_SYNTHESIZER_RX,
    SX_RX = RF_OPMODE_RECEIVER,
} sx127x_opmode_t;

typedef void (*sx127x_callback_t)(const sx127x_dt_spec_t *config, sx127x_irq_flags_t falgs);

typedef struct
{
    // all initlized by sx127x
    const sx127x_dt_spec_t *spec;
    gpio_user_callback_t dio0_callback;
    sx127x_callback_t callback;
} sx127x_config_t;

#define DEFINE_SX127X(_l)                                                            \
    const sx127x_dt_spec_t _l = {.spi = SPI_DT_SPEC_GET(L(_l), SPI_WORD_SET(8), 50), \
                                 .reset = GPIO_DT_SPEC_GET(L(_l), reset_gpios),      \
                                 .dio0 = GPIO_DT_SPEC_GET(L(_l), dio_gpios)}

#define SX127X_RESET_TIME_MS 20

int sx127x_read_reg(const sx127x_dt_spec_t *sx127x, uint8_t reg, uint8_t *val);
int sx127x_write_reg(const sx127x_dt_spec_t *sx127x, uint8_t reg, uint8_t val);
int sx127x_set_freq(const sx127x_dt_spec_t *sx127x, uint32_t freq);

int sx127x_get_opmode(const sx127x_dt_spec_t *sx127x, uint8_t *mode);
int sx127x_set_opmode(const sx127x_dt_spec_t *sx127x, uint8_t mode);
int sx127x_update_modem_mode(const sx127x_dt_spec_t *sx127x, uint8_t mode);

int sx127x_configure(const sx127x_dt_spec_t *spec, sx127x_config_t *config, sx127x_callback_t callback);
int sx127x_setup_modem(const sx127x_dt_spec_t *sx12x, sx127x_modem_config_t *modem_config);

int sx127x_get_rx_length(const sx127x_dt_spec_t *sx127x, uint8_t *length);
int sx127x_receive(const sx127x_dt_spec_t *sx127x, void *buffer, uint8_t length);
int sx127x_transmit(const sx127x_dt_spec_t *sx127x, void *buffer, uint8_t length);

#endif