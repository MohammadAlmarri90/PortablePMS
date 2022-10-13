/*
 * Author: Jefflongo
 * Integrated to STM32: Mohammad Almarri
 */

#include <stdbool.h>
#include <stdint.h>
#include "main.h"



#define BAT_LOW_PERCENT 15 // 1 to 32

typedef uint16_t max17048_voltage_t;

typedef uint8_t max17048_soc_t;

enum {
    MAX_ALERT_RESET = 0x01,
    MAX_ALERT_OVERVOLTED = 0x02,
    MAX_ALERT_UNDERVOLTED = 0x04,
    MAX_ALERT_VOLTAGE_RESET = 0x80,
    MAX_ALERT_SOC_LOW = 0x10,
    MAX_ALERT_SOC_CHANGE = 0x20,
};
typedef uint8_t max17048_alert_t;

//All registers are written and read as 16-bits (P. 10)

bool i2c_master_read_u16(I2C_HandleTypeDef *i2cHandle, uint8_t device, uint8_t reg, uint16_t *buf );

bool max17048_is_present(I2C_HandleTypeDef *i2cHandle);

__attribute__((nonnull)) bool max17048_get_vcell(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t* mv);

__attribute__((nonnull)) bool max17048_get_soc(I2C_HandleTypeDef *i2cHandle, max17048_soc_t* percent);

bool max17048_set_bat_low_soc(I2C_HandleTypeDef *i2cHandle, max17048_soc_t percent);

bool max17048_set_undervolted_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv);

bool max17048_set_overvolted_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv);

bool max17048_set_reset_voltage(I2C_HandleTypeDef *i2cHandle, max17048_voltage_t mv);

bool max17048_set_soc_change_alert(I2C_HandleTypeDef *i2cHandle, bool enable);

bool max17048_set_voltage_reset_alert(I2C_HandleTypeDef *i2cHandle, bool enable);

bool max17048_clear_alerts(I2C_HandleTypeDef *i2cHandle);

__attribute__((nonnull)) bool max17048_get_alerts(I2C_HandleTypeDef *i2cHandle, max17048_alert_t* status);
