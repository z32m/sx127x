#include <sx127x.h>
#include <xlog.h>

#ifndef SX127x_DEBUG
LOG_MODULE_REGISTER(sx127x, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(sx127x, LOG_LEVEL_DBG);
#endif

BITS_SINGLE(opmode_mode, 7,
            DESC(1, "LoRa"),
            DESC(0, "FSK/OOK"));

BITS_SINGLE(opmode_freq, 3,
            DESC(0, "HF"),
            DESC(1, "LF"));

BITS_GROUP(opmode_modulation_type, 5, 6,
           DESC(0b00, "FSK"),
           DESC(0b01, "OOK"));

BITS_GROUP(opmode_state, 0, 2,
           DESC(0b000, "SLEEP"),
           DESC(0b001, "STDBY"),
           DESC(0b011, "TX"),
           DESC(0b010, "FS-TX"),
           DESC(0b100, "FS-RX"),
           DESC(0b101, "RX-CONT"),
           DESC(0b110, "RX-SINGLE"));

BITS_VALUE(sx127x_reg_opmode, opmode_mode, opmode_modulation_type, opmode_freq, opmode_state);

BITS_BIT(irqflags, timeout, 7);
BITS_BIT(irqflags, rx_done, 6);
BITS_BIT(irqflags, payload_crc_error, 5);
BITS_BIT(irqflags, valid_header, 4);
BITS_BIT(irqflags, tx_done, 3);
BITS_BIT(irqflags, cad_done, 2);
BITS_BIT(irqflags, tifhss_changed_channelmeout, 1);
BITS_BIT(irqflags, cad_detected, 0);

BITS_VALUE(sx127x_reg_irqflags,
           irqflags_timeout,
           irqflags_rx_done,
           irqflags_payload_crc_error,
           irqflags_valid_header,
           irqflags_tx_done,
           irqflags_cad_done,
           irqflags_tifhss_changed_channelmeout,
           irqflags_cad_detected);

int sx127x_read_reg(const sx127x_dt_spec_t *sx127x, uint8_t reg, uint8_t *val)
{
    const spi_t *spi = &sx127x->spi;

    return spi_write_read(spi, &reg, 1, val, 1);
}

int sx127x_write_reg(const sx127x_dt_spec_t *sx127x, uint8_t reg, uint8_t val)
{
    const spi_t *spi = &sx127x->spi;

    LOG_DBG("setting reg: 0x%x = %x", reg, val);
    uint8_t buf[2] = {reg | 0x80, val};
    return spi_write(spi, buf, 2);
}

int sx127x_update_reg(const sx127x_dt_spec_t *sx127x, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    int err = sx127x_read_reg(sx127x, reg, &v);
    if (err)
        return err;

    return sx127x_write_reg(sx127x, reg, (v & mask) | val);
}

#define SX1276_XTAL_FREQ 32000000UL
#define SX1276_PLL_STEP_SHIFT_AMOUNT (8)
#define SX1276_PLL_STEP_SCALED (SX1276_XTAL_FREQ >> (19 - SX1276_PLL_STEP_SHIFT_AMOUNT))

static uint32_t sx127x_convert_freq_pll(uint32_t freqInHz)
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX1276_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX1276_PLL_STEP_SCALED;
    stepsFrac = freqInHz - (stepsInt * SX1276_PLL_STEP_SCALED);

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return (stepsInt << SX1276_PLL_STEP_SHIFT_AMOUNT) +
           (((stepsFrac << SX1276_PLL_STEP_SHIFT_AMOUNT) + (SX1276_PLL_STEP_SCALED >> 1)) /
            SX1276_PLL_STEP_SCALED);
}

int sx127x_set_freq(const sx127x_dt_spec_t *sx127x, uint32_t freq)
{
    uint32_t freqInPllSteps = sx127x_convert_freq_pll(freq);

    fallback(1, sx127x_write_reg, sx127x, REG_FRFMSB, (uint8_t)((freqInPllSteps >> 16) & 0xFF));
    fallback(1, sx127x_write_reg, sx127x, REG_FRFMID, (uint8_t)((freqInPllSteps >> 8) & 0xFF));
    fallback(1, sx127x_write_reg, sx127x, REG_FRFLSB, (uint8_t)(freqInPllSteps & 0xFF));

    return 0;
}

int sx127x_set_opmode(const sx127x_dt_spec_t *sx127x, uint8_t mode)
{
    return sx127x_update_reg(sx127x, REG_OPMODE, RF_OPMODE_MASK, mode);
}

int sx127x_update_modem_mode(const sx127x_dt_spec_t *sx127x, uint8_t mode)
{
    return sx127x_update_reg(sx127x, REG_OPMODE, RF_OPMODE_LONGRANGEMODE_MASK, mode);
}

int sx127x_update_freq_mode(const sx127x_dt_spec_t *sx127x, uint8_t mode)
{
    return sx127x_update_reg(sx127x, REG_OPMODE, RFLR_OPMODE_FREQMODE_ACCESS_MASK, mode);
}

int sx127x_reset(const sx127x_dt_spec_t *config)
{
    const gpio_dt_spec_t *reset = &config->reset;
    // gpio_pin_set_dt(reset, 1);

    fallback(1, gpio_pin_set_dt, reset, 1);
    k_msleep(SX127X_RESET_TIME_MS);
    fallback(1, gpio_pin_set_dt, reset, 0);

    return 0;
}

#define EVENTS_MAX 4
K_PIPE_DEFINE(sx127x_events, EVENTS_MAX * sizeof(void *), ALIGN);
static size_t written;

void sx127x_irq(const struct device *port, gpio_callback_t *cb, gpio_port_pins_t pins)
{
    const gpio_user_callback_t *t = CONTAINER_OF(cb, gpio_user_callback_t, gpio_cb);
    sx127x_config_t *config = t->data;
    SURE(k_pipe_put, &sx127x_events, &config, sizeof(config), &written, sizeof(config), K_NO_WAIT);
}

void sx127x_irq_thread()
{
    size_t read;
    uint8_t flags;

    while (true)
    {
        sx127x_config_t *config;
        SURE(k_pipe_get, &sx127x_events, &config, sizeof(config), &read, sizeof(config), K_FOREVER);
        SURE(sx127x_read_reg, config->spec, REG_LR_IRQFLAGS, &flags);
        config->callback(config->spec, flags);
        SURE(sx127x_write_reg, config->spec, REG_LR_IRQFLAGS, 0xff);
    }
}
K_THREAD_DEFINE(sx127x_irq_thread_id, 1024, sx127x_irq_thread, NULL, NULL, NULL, 5, 0, 0);

int sx127x_configure(const sx127x_dt_spec_t *spec, sx127x_config_t *config, sx127x_callback_t callback)
{
    config->spec = spec;
    config->callback = callback;

    fallback(1, gpio_pin_configure_dt, &spec->reset, GPIO_OUTPUT);
    fallback(1, sx127x_reset, config->spec);

    const gpio_dt_spec_t *dio0 = &spec->dio0;
    fallback(1, gpio_pin_configure_dt, dio0, GPIO_INPUT);
    fallback(1, gpio_pin_interrupt_configure_dt, dio0, GPIO_INT_EDGE_RISING);
    config->dio0_callback.data = config;

    gpio_init_callback(&config->dio0_callback.gpio_cb, sx127x_irq, BIT(dio0->pin));
    fallback(1, gpio_add_callback, dio0->port, (gpio_callback_t *)&config->dio0_callback);

    return 0;
}

int sx127x_setup_modem(const sx127x_dt_spec_t *sx12x, sx127x_modem_config_t *modem_config)
{
    const sx127x_dt_spec_t *spi = sx12x;
    sx127x_modem_config_t *config = modem_config;

    fallback(1, sx127x_set_opmode, spi, RF_OPMODE_SLEEP);
    fallback(1, sx127x_update_modem_mode, spi, RFLR_OPMODE_LONGRANGEMODE_ON);
    fallback(1, sx127x_update_freq_mode, spi, RFLR_OPMODE_FREQMODE_ACCESS_LF);
    fallback(1, sx127x_set_opmode, spi, RF_OPMODE_STANDBY);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_BW_MASK, config->bandwidth);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_CODINGRATE_MASK, config->coding_rate);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK, config->implicit_header);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SF_MASK, config->data_rate);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK, RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK, RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK, RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON);

    fallback(1, sx127x_write_reg, spi, REG_LR_SYMBTIMEOUTLSB, RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT);

    fallback(1, sx127x_write_reg, spi, REG_LR_PREAMBLEMSB, (uint8_t)((config->preamble_len >> 8) & 0xFF));
    fallback(1, sx127x_write_reg, spi, REG_LR_PREAMBLELSB, (uint8_t)(config->preamble_len & 0xFF));
    fallback(1, sx127x_set_freq, spi, config->freq);

    // fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON);
    fallback(1, sx127x_write_reg, spi, REG_LR_PAYLOADLENGTH, config->packet_len);
    // ? RegMaxPayloadLength
    // ? RegSyncWord

    // if (config->data_rate == SF_6)
    // {
    //     fallback(1, sx127x_update_reg, spi, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_MASK, RFLR_DETECTIONOPTIMIZE_SF6);
    //     fallback(1, sx127x_write_reg, spi, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    // }
    // else
    // {
    //     fallback(1, sx127x_update_reg, spi, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_MASK, RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
    //     fallback(1, sx127x_write_reg, spi, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    // }

    return 0;
}

int sx127x_get_opmode(const sx127x_dt_spec_t *sx127x, uint8_t *mode)
{
    return sx127x_read_reg(sx127x, REG_OPMODE, mode);
}

int inline sx127x_get_rx_length(const sx127x_dt_spec_t *sx127x, uint8_t *length)
{
    return sx127x_read_reg(sx127x, REG_LR_RXNBBYTES, length);
}

int sx127x_receive(const sx127x_dt_spec_t *sx127x, uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    fallback(1, sx127x_read_reg, sx127x, REG_LR_FIFORXCURRENTADDR, &i);
    fallback(1, sx127x_write_reg, sx127x, REG_LR_FIFOADDRPTR, i);
    i = 0;
    while (i < length)
    {
        fallback(1, sx127x_read_reg, sx127x, REG_LR_FIFO, buffer + i);
        i++;
    }

    return 0;
}

int sx127x_transmit(const sx127x_dt_spec_t *sx127x, uint8_t *buffer, uint8_t length)
{
    uint8_t tx_base;
    fallback(1, sx127x_read_reg, sx127x, REG_LR_FIFOTXBASEADDR, &tx_base);

    int i = 0;
    while (i < length)
    {
        fallback(1, sx127x_write_reg, sx127x, REG_LR_FIFOADDRPTR, tx_base + i);
        fallback(1, sx127x_write_reg, sx127x, REG_LR_FIFO, buffer + i);
        i++;
    }

    fallback(1, sx127x_set_opmode, sx127x, RF_OPMODE_TRANSMITTER);
    return 0;
}