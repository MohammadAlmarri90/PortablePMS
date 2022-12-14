#include "max17048.h"


#include <stdint.h>

// MAX17048 i2c address
#define MAX_ADDR 0x36

// MAX17048 registers
#define VCELL 0x02     // R/-: ADC measurement of VCELL, 78.125uV/cell
#define SOC 0x04       // R/-: Battery state of charge, 1%/256
#define MODE 0x06      // -/W: Current operating mode, enable sleep
#define VERSION 0x08   // R/-: Production version
#define HIBRT 0x0A     // R/W: Hibernation thresholds
#define CONFIG 0x0C    // R/W: Compensation, toggle sleep, alert masks, config
#define VALRT 0x14     // R/W: Voltage level to generate alert
#define CRATE 0x16     // R/-: Charge/discharge rate, 0.208%/hr
#define VRESET_ID 0x18 // R/W: VCELL for chip reset
#define STATUS 0x1A    // R/W: Over/undervoltage, SOC change/low, reset alerts
#define TABLE 0x40     // -/W: Configures battery parameters
#define CMD 0xFE       // R/W: POR command

// MAX17048 masks/constants
#define VERSION_MSK 0xFFF0
#define PART_NUMBER 0x0010

#define BAT_LOW_POS 0
#define BAT_LOW_MSK (0x001F << BAT_LOW_POS)
#define BAT_LOW_MIN 1
#define BAT_LOW_MAX 32

#define ALRT_BIT_POS 5
#define ALRT_BIT_MSK (0x0001 << ALRT_BIT_POS)

#define ALSC_BIT_POS 6
#define ALSC_BIT_MSK (0x0001 << ALSC_BIT_POS)

#define VALRT_MAX_POS 0
#define VALRT_MAX_MSK (0x00FF << VALRT_MAX_POS)
#define VALRT_MIN_POS 8
#define VALRT_MIN_MSK (0x00FF << VALRT_MIN_POS)
#define VALRT_RESOLUTION 20

#define VRESET_POS 9
#define VRESET_MSK (0x007F << VRESET_POS)
#define VRESET_RESOLUTION 40

#define ENVR_BIT_POS 14
#define ENVR_BIT_MSK (0x0001 << ENVR_BIT_POS)

#define ALRT_STATUS_POS 8
#define ALRT_STATUS_MSK (0x003F << ALRT_STATUS_POS)

#define VCELL_TO_MV(vcell) ((vcell * 5) >> 6)

// MAX17048 uses big endian register layout
#define SWAP16(x) ((uint16_t)(((x) << 8) | ((x) >> 8)))


bool i2c_master_read_u16(I2C_HandleTypeDef *i2cHandle, uint8_t device, uint8_t reg, uint16_t *buf )
{
	if ( HAL_I2C_Mem_Read( i2cHandle, device, reg, I2C_MEMADD_SIZE_16BIT, buf, 2, 10 ) != HAL_OK)
	{
		return 0;
	}

	return 1;
}

bool i2c_master_write_u16(I2C_HandleTypeDef *i2cHandle, uint8_t device, uint8_t reg, uint16_t *buf )
{
	if ( HAL_I2C_Mem_Write( i2cHandle, device, reg, I2C_MEMADD_SIZE_16BIT, buf, 2, 10 ) != HAL_OK)
	{
		return 0;
	}

	return 1;
}


static inline bool read_reg(I2C_HandleTypeDef *i2cHandle, uint8_t reg, uint16_t* out) {
    if (!i2c_master_read_u16(i2cHandle, MAX_ADDR, reg, out)) {
        return false;
    }

    *out = SWAP16(*out);
    return true;
}

static inline bool write_reg(I2C_HandleTypeDef *i2cHandle, uint8_t reg, uint16_t data) {
    return i2c_master_write_u16(i2cHandle, MAX_ADDR, reg, SWAP16(data));
}

static inline bool modify_reg(I2C_HandleTypeDef *i2cHandle, uint8_t reg, uint16_t data, uint16_t mask) {
    uint16_t buf;
    if (!read_reg(i2cHandle, reg, &buf)) {
        return false;
    }

    buf = (buf & ~mask) | (data & mask);
    return write_reg(i2cHandle, reg, buf);
}

bool max17048_is_present(I2C_HandleTypeDef *i2cHandle) {
    uint16_t data;
    if (!read_reg(i2cHandle, VERSION, &data)) {
        return false;
    }

    return ((data & VERSION_MSK) == PART_NUMBER);
}

bool max17048_get_vcell(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t* mv) {
    uint16_t data;
    if (!read_reg(i2cHandle, VCELL, &data)) {
        return false;
    }

    *mv = (max17048_voltage_t)(VCELL_TO_MV(data));
    return true;
}

bool max17048_get_soc(I2C_HandleTypeDef *i2cHandle, max17048_soc_t* percent) {
    uint16_t data;
    if (!read_reg(i2cHandle, SOC, &data)) {
        return false;
    }

    *percent = (max17048_soc_t)(data >> 8);
    return true;
}

bool max17048_set_bat_low_soc(I2C_HandleTypeDef *i2cHandle, max17048_soc_t percent) {
    if (percent < BAT_LOW_MIN || percent > BAT_LOW_MAX) {
        return false;
    }
    uint16_t data = (uint16_t)((BAT_LOW_MAX - (percent % BAT_LOW_MAX)) & BAT_LOW_MSK);

    return modify_reg(i2cHandle, CONFIG, data, BAT_LOW_MSK);
}

bool max17048_set_undervolted_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv) {
    uint16_t data = (uint16_t)(((mv / VALRT_RESOLUTION) << VALRT_MIN_POS) & VALRT_MIN_MSK);

    return modify_reg(i2cHandle, VALRT, data, VALRT_MIN_MSK);
}

bool max17048_set_overvolted_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv) {
    uint16_t data = (uint16_t)(((mv / VALRT_RESOLUTION) << VALRT_MAX_POS) & VALRT_MAX_MSK);

    return modify_reg(i2cHandle, VALRT, data, VALRT_MAX_MSK);
}

bool max17048_set_reset_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv) {
    uint16_t data = (uint16_t)(((mv / VRESET_RESOLUTION) << VRESET_POS) & VRESET_MSK);

    return modify_reg(i2cHandle, VRESET_ID, data, VRESET_MSK);
}

bool max17048_set_soc_change_alert(I2C_HandleTypeDef *i2cHandle, bool enable) {
    uint16_t data = (uint16_t)((enable << ALSC_BIT_POS) & ALSC_BIT_MSK);

    return modify_reg(i2cHandle, CONFIG, data, ALSC_BIT_MSK);
}

bool max17048_set_voltage_reset_alert(I2C_HandleTypeDef *i2cHandle, bool enable) {
    uint16_t data = (uint16_t)((enable << ENVR_BIT_POS) & ENVR_BIT_MSK);

    return modify_reg(i2cHandle, STATUS, data, ENVR_BIT_MSK);
}

bool max17048_clear_alerts(I2C_HandleTypeDef *i2cHandle) {
    bool ok = true;

    if (ok) ok = modify_reg(i2cHandle, STATUS, 0, ALRT_STATUS_MSK);
    if (ok) ok = modify_reg(i2cHandle, CONFIG, 0, ALRT_BIT_MSK);

    return ok;
}

bool max17048_get_alerts(I2C_HandleTypeDef *i2cHandle, max17048_alert_t* alerts) {
    bool ok = true;
    uint16_t data;

    if (ok) ok = read_reg(i2cHandle, STATUS, &data);
    if (ok) ok = max17048_clear_alerts(i2cHandle);

    *alerts = (max17048_alert_t)((data & ALRT_STATUS_MSK) >> ALRT_STATUS_POS);
    return true;
}
