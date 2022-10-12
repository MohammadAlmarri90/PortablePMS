/*
 * BQ24295.c
 *
 *  Created on: Oct 12, 2022
 *      Author: Mohammad.Almarri
 */


void BQ_Write(unsigned char reg, unsigned char data) {

	if ( HAL_I2C_Mem_Write( &hi2c2, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorWrite;
	}

}

unsigned short BQ_Read(unsigned char reg) {
    unsigned char data;

	if ( HAL_I2C_Mem_Read( &hi2c2, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorRead;
	}

    return data;

}
