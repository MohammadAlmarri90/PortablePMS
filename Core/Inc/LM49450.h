/*
 * LM49450.h
 *
 *  Created on: Dec 11, 2021
 *      Author: Mohammad Almarri
 *
 *      Taken from Gunnar's LM49450 implementation on the PIC16f
 *      and ported to the STM32
 */
#if (0)
#ifndef INC_LM49450_H_
#define INC_LM49450_H_

#include "main.h"


/*
 * DEFINES
 */

#define LM49450_I2C_ADDR	(0x7D << 1)	/* LM49450 Device address (p. 22), Shifted to left by 1 as required by the HAL driver*/

/*
 * Typedefs
 */

extern uint8_t mute_config;
extern I2C_HandleTypeDef hi2c2;	//Double check your i2c handler
extern uint8_t errors;
/*
 * Functions
 */

void LM49450_Write (uint8_t reg, uint8_t *data);
uint8_t LM49450_Wii_init();
uint8_t LM49450_PS2_init();
uint8_t LM49450_analog_init();

#endif /* INC_LM49450_H_ */
