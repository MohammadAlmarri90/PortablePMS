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

void BQ_INIT();
void BQ_update();


unsigned short BQ_Read(unsigned char reg);
void BQ_Write(unsigned char reg, unsigned char data);

#endif /* INC_BQ24295_H_ */
