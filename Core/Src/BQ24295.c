/*
 * BQ24295.c
 *
 *  Created on: Oct 12, 2022
 *      Author: Mohammad.Almarri
 */
#include "BQ24295.h"

enum BQ_Registers {
	InputSourceControlReg,						//RW
	PowerOnConfigReg,							//RW
	ChargeCurrentControlReg,					//RW
	PrechargeTerminationCurrentControlReg,		//RW
	ChargeVoltageControlReg,					//RW
	ChargeTerminationTimerControlReg,			//RW
	BoostVoltageThermalRegulationControlReg,	//RW
	MiscOperationControlReg,					//RW
	SystemStatusReg,							//R
	NewFaultReg,								//R
	PartStatusReg,								//R
};

struct BQ_Register_fields{

    struct {
        uint8_t IINLIM: 3;
        uint8_t VINDPM: 4;
        uint8_t EN_HIZ: 1;
    }REG00;

    struct {
        uint8_t Reserved: 1;
        uint8_t SYS_MIN: 3;
        uint8_t CHG_CONFIG: 1;
        uint8_t OTG_CONFIG: 1;
        uint8_t I2CWDTIMER: 1;
        uint8_t RegisterReset: 1;
    }REG01;

    struct {
        uint8_t FORCE_20PCT: 1;
        uint8_t BCOLD: 1;
        uint8_t ICHG: 6;
    }REG02;

    struct {
        uint8_t ITERM: 4;
        uint8_t IPRECHG: 4;
    }REG03;

    struct {
        uint8_t VRECHG: 1;
        uint8_t BATLOWV: 1;
        uint8_t VREG: 6;
    }REG04;

    struct {
        uint8_t Reserved: 1;
        uint8_t CHG_TIMER: 2;
        uint8_t EN_TIMER: 1;
        uint8_t WATCHDOG: 2;
        uint8_t Reserved2: 1;
        uint8_t EN_TERM: 1;
    }REG05;

    struct {
        uint8_t TREG: 2;
        uint8_t BHOT: 2;
        uint8_t BOOSTV: 4;
    }REG06;

    struct {
        uint8_t INT_MASK: 2;
        uint8_t Reserved: 3;
        uint8_t BATFET_Disable: 1;
        uint8_t TMR2X_EN: 1;
        uint8_t DPDM_EN: 1;
    }REG07;

};




void BQ_Write(unsigned char reg, unsigned char data) {

	if ( HAL_I2C_Mem_Write( &hi2c1, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorWrite;
	}

}

unsigned short BQ_Read(unsigned char reg) {
    unsigned char data;

	if ( HAL_I2C_Mem_Read( &hi2c1, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorRead;
	}

    return data;

}

void BQ_INIT()
{
	struct BQ_Register_fields BQ;


}
