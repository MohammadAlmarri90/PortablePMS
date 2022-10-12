/*
 * BQ24295.h
 *
 *  Created on: Oct 12, 2022
 *      Author: super
 */

#ifndef INC_BQ24295_H_
#define INC_BQ24295_H_

#include "main.h"

/*			DEFINES			*/
#define BQ_ADDR (0x6B << 1)

/*			EXTERNS			*/
extern I2C_HandleTypeDef hi2c1;
extern uint8_t GLOBAL_errors;


/*			PROTOTYPES		*/
void BQ_init();
void BQ_update();
unsigned short BQ_Read(unsigned char reg);
void BQ_Write(unsigned char reg, unsigned char data);

#endif /* INC_BQ24295_H_ */
