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

    struct REG00{
        uint8_t IINLIM: 3;
        uint8_t VINDPM: 4;
        uint8_t EN_HIZ: 1;
    }REG00;

    struct REG01{
        uint8_t Reserved: 1;
        uint8_t SYS_MIN: 3;
        uint8_t CHG_CONFIG: 1;
        uint8_t OTG_CONFIG: 1;
        uint8_t I2CWDTIMER: 1;
        uint8_t RegisterReset: 1;
    }REG01;

    struct REG02{
        uint8_t FORCE_20PCT: 1;
        uint8_t BCOLD: 1;
        uint8_t ICHG: 6;
    }REG02;

    struct REG03{
        uint8_t ITERM: 4;
        uint8_t IPRECHG: 4;
    }REG03;

    struct REG04{
        uint8_t VRECHG: 1;
        uint8_t BATLOWV: 1;
        uint8_t VREG: 6;
    }REG04;

    struct REG05{
        uint8_t Reserved: 1;
        uint8_t CHG_TIMER: 2;
        uint8_t EN_TIMER: 1;
        uint8_t WATCHDOG: 2;
        uint8_t Reserved2: 1;
        uint8_t EN_TERM: 1;
    }REG05;

    struct REG06{
        uint8_t TREG: 2;
        uint8_t BHOT: 2;
        uint8_t BOOSTV: 4;
    }REG06;

    struct REG07{
        uint8_t INT_MASK: 2;
        uint8_t Reserved: 3;
        uint8_t BATFET_Disable: 1;
        uint8_t TMR2X_EN: 1;
        uint8_t DPDM_EN: 1;
    }REG07;

};




void BQ_Write(uint8_t reg, uint8_t data) {

	if ( HAL_I2C_Mem_Write( &hi2c1, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorWrite;
	}

}

uint8_t BQ_Read(uint8_t reg) {
    unsigned char data;

	if ( HAL_I2C_Mem_Read( &hi2c1, BQ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10 ) != HAL_OK)
	{
		GLOBAL_errors = I2CErrorRead;
	}

    return data;

}

bool BQ_Init()
{
	struct BQ_Register_fields BQ;

	//REG00
	BQ.REG00.IINLIM = 0b111;	//3A input
	BQ.REG00.VINDPM = 0b1011;	//4.76V
	BQ.REG00.EN_HIZ = 1;

	//REG01
	BQ.REG01.SYS_MIN = 0b011;	//3.3V minimum voltage cutoff
	BQ.REG01.CHG_CONFIG = 1;	//enable charge
	BQ.REG01.OTG_CONFIG = 0;
	BQ.REG01.I2CWDTIMER = 0;	//disable WD
	BQ.REG01.RegisterReset = 0;

	//REG02
	BQ.REG02.FORCE_20PCT = 0;
	BQ.REG02.BCOLD = 0;
	BQ.REG02.ICHG = 0b101111;	//Fast Charging current is 3008mAh

	//REG03
	BQ.REG03.ITERM = 0b0011;	//Termination current is 384mAh
	BQ.REG03.IPRECHG = 0b0001;	//Precharge current is 128mAh

	//REG04
	BQ.REG04.VRECHG = 0;
	BQ.REG04.BATLOWV = 1;		//Battery is precharged until 3.0v then changed to fast charge
	BQ.REG04.VREG = 0b101100;	//Battery is full at 4.208v

	//REG05
	BQ.REG05.CHG_TIMER = 0b01;	//If enabled, it will charge for 8 hours
	BQ.REG05.EN_TIMER = 0;		//disable charging safety timer
	BQ.REG05.WATCHDOG = 0b00;	//disable watchdog
	BQ.REG05.EN_TERM = 1;

	//REG06
	BQ.REG06.TREG = 0b00;		//Thermal reg at 60C degrees
	BQ.REG06.BHOT = 0b00;
	BQ.REG06.BOOSTV= 0b1001;	//not needed

	//REG07
	BQ.REG07.INT_MASK = 0b11;	//Enable pin Inturrupts
	BQ.REG07.BATFET_Disable = 0;//to Force BATFET off, turn this ON
	BQ.REG07.TMR2X_EN = 1;
	BQ.REG07.DPDM_EN = 0;

	if(!BQ_IsPresent())
	{
		return false;
	}
	BQ_Write((uint8_t *)InputSourceControlReg, *(uint8_t *)&BQ.REG00);
	BQ_Write((uint8_t *)PowerOnConfigReg, *(uint8_t *)&BQ.REG01);
	BQ_Write((uint8_t *)ChargeCurrentControlReg, *(uint8_t *)&BQ.REG02);
	BQ_Write((uint8_t *)PrechargeTerminationCurrentControlReg, *(uint8_t *)&BQ.REG03);
	BQ_Write((uint8_t *)ChargeVoltageControlReg, *(uint8_t *)&BQ.REG04);
	BQ_Write((uint8_t *)ChargeTerminationTimerControlReg, *(uint8_t *)&BQ.REG05);
	BQ_Write((uint8_t *)BoostVoltageThermalRegulationControlReg, *(uint8_t *)&BQ.REG06);
	BQ_Write((uint8_t *)MiscOperationControlReg, *(uint8_t *)&BQ.REG07);

	return true;
}

bool BQ_IsPresent()
{
	if(GLOBAL_errors != 0)
	{
		return false;
	}

	uint8_t reg = BQ_Read(PartStatusReg);

	reg = (reg >>5);

	if(BQ_Read(PartStatusReg) == 0b110)
	{
		return true;
	}else
	{
		GLOBAL_errors = BQNotPresentOrNotCorrectPart;
		return false;
	}
}


bool BQ_IsCharging()
{
	uint8_t SystemStatus 	= BQ_Read(SystemStatusReg);
	uint8_t chargeStatus 	= (SystemStatus >> 4) & 0b11;
	if(chargeStatus == 0b01 || chargeStatus == 0b10)	//Pre-Charging or Fast-Charging
	{
		return true;
	}
	return false;

}



