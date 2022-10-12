/*
 * BQ24295.h
 *
 *  Created on: Oct 12, 2022
 *      Author: super
 */

#ifndef INC_BQ24295_H_
#define INC_BQ24295_H_

#include "main.h"

#define BQ_ADDR (0x6B << 1)
extern I2C_HandleTypeDef hi2c1;
extern uint8_t GLOBAL_errors;

void BQ_Init();

uint8_t BQ_Read(uint8_t reg);
void BQ_Write(uint8_t reg, uint8_t data);
bool BQ_IsPresent();
uint8_t BQ_IsCharging();
#endif /* INC_BQ24295_H_ */
