/*
 * LM49450.c
 *
 *  Created on: Dec 11, 2021
 *      Author: super
 */

#if (0)
#include "LM49450.h"


void LM49450_Write (uint8_t reg, uint8_t *data)
{
	if(HAL_I2C_Mem_Write(&hi2c2, LM49450_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10) != HAL_OK)
	{
		errors = 1;
	}
}




uint8_t LM49450_Wii_init()
{
    uint8_t reg0_Wii = 0b00101001;
    uint8_t temp;
    LM49450_Write(0x00, &reg0_Wii );

    temp = 0b00000001;
    LM49450_Write(0x01, &temp ); // /1

    temp = 0x4B;
    LM49450_Write(0x02, &temp );      // 75 recommended value for 12.228MHz

    temp = 0b01110001;
    LM49450_Write(0x03, &temp );

    temp = 0b00000000;
    LM49450_Write(0x04, &temp );

    return reg0_Wii;
}


uint8_t LM49450_PS2_init()
{
    uint8_t reg0_PS2 = 0b00101001;
    uint8_t temp;
    LM49450_Write(0x00, &reg0_PS2);

    temp = 0b00000010;
    LM49450_Write(0x01, &temp );

    temp = 0x4B;
    LM49450_Write(0x02, &temp );

    temp = 0b01110010;
    LM49450_Write(0x03, &temp );

    temp = 0b00000000;
    LM49450_Write(0x04, &temp );

    return reg0_PS2;
}


uint8_t LM49450_analog_init()
{
    uint8_t reg0_analog = 0b00101011;

    LM49450_Write(0x00, &reg0_analog);

    return reg0_analog;
}

#endif
