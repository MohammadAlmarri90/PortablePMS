/*
 * BQ25895M.h
 *
 *  Created on: May 11, 2022
 *      Author: super
 */

#ifndef INC_BQ25895M_H_
#define INC_BQ25895M_H_

#include "main.h"
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

#define BQ_ADDR (0x6A << 1)

extern uint8_t errors;
extern uint8_t VBUS_CHRG_STATE[2];
extern uint8_t BATTERY_VOLTAGE;
extern uint8_t BQ_adc_state;

void BQ_Write(unsigned char reg, unsigned char data);
unsigned short BQ_Read(unsigned char reg);
void BQ_INIT();
void BQ_CONFIG_INIT();
void BQ_update();

enum {
    BQ_STATE_NOT_CHARGING = 0b00,
    BQ_STATE_PRECHARGE = 0b01,
    BQ_STATE_FAST_CHARGE = 0b10,
    BQ_STATE_TERMINATED = 0b11,
};

#endif /* INC_BQ25895M_H_ */
