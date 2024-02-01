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
           DESC(0b010, "FSTX"),
           DESC(0b011, "TX"),
           DESC(0b010, "FSTX"),
           DESC(0b100, "FSRX"),
           DESC(0b101, "RX"));

BITS_VALUE(sx127x_reg_opmode, opmode_mode, opmode_modulation_type, opmode_freq, opmode_state);

int sx127x_read_reg(const spi_t *spi, uint8_t reg, uint8_t *val)
{
    return spi_write_read(spi, &reg, 1, val, 1);
}

int sx127x_write_reg(const spi_t *spi, uint8_t reg, uint8_t val)
{
    LOG_DBG("setting reg: 0x%x = %x", reg, val);
    uint8_t buf[2] = {reg | 0x80, val};
    return spi_write(spi, buf, 2);
}

int sx127x_update_reg(const spi_t *spi, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    int err = sx127x_read_reg(spi, reg, &v);
    if (err)
        return err;

    return sx127x_write_reg(spi, reg, (v & mask) | val);
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

int sx127x_set_freq(const spi_t *spi, uint32_t freq)
{
    uint32_t freqInPllSteps = sx127x_convert_freq_pll(freq);

    fallback(1, sx127x_write_reg, spi, REG_FRFMSB, (uint8_t)((freqInPllSteps >> 16) & 0xFF));
    fallback(1, sx127x_write_reg, spi, REG_FRFMID, (uint8_t)((freqInPllSteps >> 8) & 0xFF));
    fallback(1, sx127x_write_reg, spi, REG_FRFLSB, (uint8_t)(freqInPllSteps & 0xFF));

    return 0;
}

int sx127x_update_opmode(const spi_t *spi, uint8_t mode)
{
    return sx127x_update_reg(spi, REG_OPMODE, RF_OPMODE_MASK, mode);
}

int sx127x_update_modem_mode(const spi_t *spi, uint8_t mode)
{
    return sx127x_update_reg(spi, REG_OPMODE, RF_OPMODE_LONGRANGEMODE_MASK, mode);
}

int sx127x_update_freq_mode(const spi_t *spi, uint8_t mode)
{
    return sx127x_update_reg(spi, REG_OPMODE, RFLR_OPMODE_FREQMODE_ACCESS_MASK, mode);
}

int sx127x_configure_lora(const spi_t *spi, sx127x_config_t *config)
{
    fallback(1, sx127x_update_opmode, spi, RF_OPMODE_SLEEP);
    fallback(1, sx127x_update_modem_mode, spi, RFLR_OPMODE_LONGRANGEMODE_ON);
    fallback(1, sx127x_update_freq_mode, spi, RFLR_OPMODE_FREQMODE_ACCESS_LF);
    fallback(1, sx127x_update_opmode, spi, RF_OPMODE_STANDBY);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_BW_MASK, config->bandwidth);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_CODINGRATE_MASK, config->coding_rate);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG1, RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK, config->implicit_header);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SF_MASK, config->data_rate);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK, RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON);
    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG2, RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK, RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB);

    fallback(1, sx127x_write_reg, spi, REG_LR_PREAMBLEMSB, (uint8_t)((config->preamble_len >> 8) & 0xFF));
    fallback(1, sx127x_write_reg, spi, REG_LR_PREAMBLELSB, (uint8_t)(config->preamble_len & 0xFF));
    fallback(1, sx127x_set_freq, spi, config->freq);

    fallback(1, sx127x_update_reg, spi, REG_LR_MODEMCONFIG3, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK, RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON);
    fallback(1, sx127x_write_reg, spi, REG_LR_PAYLOADLENGTH, config->packet_len);

    // fallback(1, sx127x_write_reg, spi, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT_MASK | RFLR_IRQFLAGS_RXDONE_MASK | RFLR_IRQFLAGS_TXDONE_MASK);

    /*

    RegFifoTxBaseAddr

    */

    // SX1276Write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

    // if (fixLen == 1)
    // {
    //     SX1276Write(REG_LR_PAYLOADLENGTH, payloadLen);
    // }

    // if (SX1276.Settings.LoRa.FreqHopOn == true)
    // {
    //     SX1276Write(REG_LR_PLLHOP, (SX1276Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
    //     SX1276Write(REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod);
    // }

    // if ((bandwidth == 9) && (SX1276.Settings.Channel > RF_MID_BAND_THRESH))
    // {
    //     // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
    //     SX1276Write(REG_LR_HIGHBWOPTIMIZE1, 0x02);
    //     SX1276Write(REG_LR_HIGHBWOPTIMIZE2, 0x64);
    // }
    // else if (bandwidth == 9)
    // {
    //     // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
    //     SX1276Write(REG_LR_HIGHBWOPTIMIZE1, 0x02);
    //     SX1276Write(REG_LR_HIGHBWOPTIMIZE2, 0x7F);
    // }
    // else
    // {
    //     // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
    //     SX1276Write(REG_LR_HIGHBWOPTIMIZE1, 0x03);
    // }

    if (config->data_rate == SF_6)
    {
        fallback(1, sx127x_update_reg, spi, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_MASK, RFLR_DETECTIONOPTIMIZE_SF6);
        fallback(1, sx127x_write_reg, spi, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    }
    else
    {
        fallback(1, sx127x_update_reg, spi, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_MASK, RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
        fallback(1, sx127x_write_reg, spi, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    }
}

int sx127x_get_opmode(const spi_t *spi, uint8_t *mode)
{
    return sx127x_read_reg(spi, REG_OPMODE, mode);
}
