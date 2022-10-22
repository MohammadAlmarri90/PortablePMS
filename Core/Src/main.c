/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "BQ24295.h"
#include "max17048.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*		MAIN CONTROLS		*/

#define ENABLESLEEPMODE 			true
#define ENABLEPID					true
#define USINGLM49450				true
#define USINGMAX17048				true

/*debouncing Controls*/
#define BUTTON_DEBOUNCE_MS			100
#define BUTTON_SHORTPRESS_PERIOD	500
#define BUTTON_LONGPRESS_PERIOD		1000 //(total is (debounce + shortpress + longpress) periods
#define BUTTON_UNINTENTIONAL_PERIOD	4000 //pressed on random object


/*		PID CONTROLS		*/
double MeasuredTemperature;
double PIDOut;
double Temperautre_SetPoint = 40;
PID_TypeDef Fan_PID;


/*		MAX17048 CONTROLS	*/
#if (USINGMAX17048)
	#define Battery_UnderVoltage	3200
	#define Battery_OverVoltage		4200
	#define Battery_ResetVoltage	2500	//Set to 2.5V as advised in datasheet (P.13)
	#define Battery_LowSOCAlert		30		//will send an alert when SOC(1-32%) decreases to assigned number
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
//TODO:
//IMPLEMENT ERRORS
uint8_t GLOBAL_errors = 0;	//Error codes in main.h


/*		FLAGS		*/
bool BQ_FLAG = false;
bool MAX_FLAG = false;

bool PowerButtonDebounced = true;
bool IsPressPeriodStart = false;
bool PowerButtonShortPress = false;
bool PowerButtonLongPress = false;
bool PowerButtonUnintentionalPress = false;

bool SystemPowerState = false;
bool InitialSystemBoot = false;
bool RequestSleep = false;
bool IsSystemCharging = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



#if (ENABLESLEEPMODE)
	void EnterSleepModeWakeOnInturrupt() {
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
	void WakeUpFromSleepMode(){
		HAL_ResumeTick();
	}
#endif



/*		INURRUPT CODE		*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BQ_INT_Pin) // BQ INT pin wake up
	{
		WakeUpFromSleepMode();
		BQ_FLAG = true;
	}
	if(GPIO_Pin == MAX_ALRT_Pin)
	{
		WakeUpFromSleepMode();
		MAX_FLAG = true;

	}
	if(GPIO_Pin == Power_Button_Pin)
	{
		if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET
				&& PowerButtonDebounced && !IsPressPeriodStart)
		{
			HAL_TIM_Base_Start_IT(&htim15);	//Start Debounce
			PowerButtonDebounced = false;
		}
		if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_SET &&
				PowerButtonDebounced && IsPressPeriodStart && PowerButtonShortPress &&
				PowerButtonLongPress && !SystemPowerState)
		{
			//All conditions met turn ON system and clear for next button
			HAL_TIM_Base_Stop_IT(&htim15);
			__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_DEBOUNCE_MS);	//Reset power button debounce period
			SystemPowerState = true;
			PowerButtonDebounced = true;
			IsPressPeriodStart = false;
			PowerButtonShortPress = false;
			PowerButtonLongPress = false;
		}
		else if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_SET &&
				PowerButtonDebounced && IsPressPeriodStart && PowerButtonShortPress &&
				PowerButtonLongPress && SystemPowerState)
		{
			//All conditions met turn OFF system and clear for next button
			HAL_TIM_Base_Stop_IT(&htim15);
			__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_DEBOUNCE_MS);	//Reset power button debounce period
			SystemPowerState = false;
			PowerButtonDebounced = true;
			IsPressPeriodStart = false;
			PowerButtonShortPress = false;
			PowerButtonLongPress = false;
		}


	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET && !PowerButtonDebounced){
		HAL_TIM_Base_Stop_IT(&htim15);
		PowerButtonDebounced = true;

	}
	if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET && !IsPressPeriodStart){

		__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_SHORTPRESS_PERIOD);
		HAL_TIM_Base_Start_IT(&htim15);
		IsPressPeriodStart = true;
		PowerButtonShortPress = false;
	}
	else if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET && !PowerButtonShortPress){
		HAL_TIM_Base_Stop_IT(&htim15);
		__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_LONGPRESS_PERIOD);
		HAL_TIM_Base_Start_IT(&htim15);
		PowerButtonShortPress = true;
		PowerButtonLongPress = false;
	}
	else if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET && !PowerButtonLongPress){
		HAL_TIM_Base_Stop_IT(&htim15);
		PowerButtonLongPress = true;
		PowerButtonUnintentionalPress = false;
		__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_UNINTENTIONAL_PERIOD);
		HAL_TIM_Base_Start_IT(&htim15);
	}
	else if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_RESET && !PowerButtonUnintentionalPress)
	{
		//unintentional button press so clear everything for next press
		HAL_TIM_Base_Stop_IT(&htim15);
		__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_DEBOUNCE_MS);	//Reset power button debounce period
		PowerButtonDebounced = true;
		IsPressPeriodStart = false;
		PowerButtonShortPress = false;
		PowerButtonLongPress = false;
	}

	if(HAL_GPIO_ReadPin(Power_Button_GPIO_Port, Power_Button_Pin) == GPIO_PIN_SET && IsPressPeriodStart)
	{
		//Cancelled press so clear everything for next press
		HAL_TIM_Base_Stop_IT(&htim15);
		__HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_DEBOUNCE_MS);	//Reset power button debounce period
		PowerButtonDebounced = true;
		IsPressPeriodStart = false;
		PowerButtonShortPress = false;
		PowerButtonLongPress = false;
		PowerButtonUnintentionalPress = false;
	}




}

#if (USINGMAX17048)

uint8_t CurrentBatteryPercentage;

	bool MAX17048_Init()
	{
		bool ok = true;
		if (ok) ok = max17048_is_present(&hi2c1);
		if (ok) ok = max17048_set_undervolted_voltage(&hi2c1, Battery_UnderVoltage);
		if (ok) ok = max17048_set_overvolted_voltage(&hi2c1, Battery_OverVoltage);
		if (ok) ok = max17048_set_reset_voltage(&hi2c1, Battery_ResetVoltage);
		if (ok) ok = max17048_set_bat_low_soc(&hi2c1, Battery_LowSOCAlert);
		if (ok) ok = max17048_set_voltage_reset_alert(&hi2c1, false);
		if (ok) ok = max17048_set_soc_change_alert(&hi2c1, false);
		if (ok) ok = max17048_clear_alerts(&hi2c1);
		return ok;
	}
#endif

static int Remap (float value, float from1, float to1, float from2, float to2) {
	return ((value - from1) / (to1 - from1) * (to2 - from2)) + from2;
}

void Set_RGB(uint8_t Red,uint8_t Green,uint8_t Blue) {
	//invert 0-100 -> 100-0 in case LED is common Anode and grounded to STM's GPIO's
	Red = Remap(Red, 0, 100, 100, 0);
	Green = Remap(Green, 0, 100, 100, 0);
	Blue = Remap(Blue, 0, 100, 100, 0);
	TIM2->CCR1 = Red;
	TIM2->CCR2 = Green;
	TIM2->CCR3 = Blue;
}


float GetADCValue(float numberOfSamples)
{
	float averageRaw = 0;
	float i = 0;
	for (i = 0; i < numberOfSamples; ++i)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		averageRaw += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}

	return (averageRaw/numberOfSamples);

}

const double BALANCE_RESISTOR   = 9710.0;
const double MAX_ADC            = 4095.0;
const double BETA               = 3974.0;
const double ROOM_TEMP          = 298.15;   // room temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 10000.0;

double readThermistor()
{

 double rThermistor = 0;            // Holds thermistor resistance value
 double tKelvin     = 0;            // Holds calculated temperature
 double tCelsius    = 0;            // Hold temperature in celsius
 double adcAverage  = 0;            // Holds the average voltage measurement

 adcAverage = GetADCValue(200);

 rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / adcAverage) - 1);

 tKelvin = (BETA * ROOM_TEMP) /
           (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));

 tCelsius = tKelvin - 273.15;  // convert kelvin to celsius

 return tCelsius;    // Return the temperature in Celsius
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#if (1)	//Init



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
#endif


  __HAL_TIM_SET_AUTORELOAD(&htim15, BUTTON_DEBOUNCE_MS);	//Set power button debounce period

  PID(&Fan_PID, &MeasuredTemperature, &PIDOut, &Temperautre_SetPoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_REVERSE);
  PID_SetMode(&Fan_PID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&Fan_PID, 500);
  PID_SetOutputLimits(&Fan_PID, 5, 100);

  HAL_Delay(70);	// For stability

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);	//Calibrate ADC for better temperature reading

#if (USINGMAX17048)
  MAX17048_Init();
#endif

  if(!BQ_Init())
  {
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	  while(1)		//if BQ not present, then warn using that it's not finding it by flashing red
	  {
		  Set_RGB(100, 0, 0);
		  HAL_Delay(250);
		  Set_RGB(0, 0, 0);
		  HAL_Delay(250);
	  }

  }

  HAL_Delay(70);	// For stability

  max17048_get_soc(&hi2c1, &CurrentBatteryPercentage);	//Get current Battery Percentage
  Set_RGB( 100, 100, 100 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(BQ_FLAG)
	  {
		  BQ_FLAG = false;	//clear flag
		  /*
		   * TODO:
		   * something with the BQ INT
		   */
		  IsSystemCharging = BQ_IsCharging();
	  }

#if (USINGMAX17048)
	  if(MAX_FLAG)
	  {
		  MAX_FLAG = false;	//clear flag
	  }

#endif

	  if(SystemPowerState)
	  {
		  /*
		   * While system is running,the code below will always run
		   */

		  if(!InitialSystemBoot)	//Start a boot sequence once
		  {
			  InitialSystemBoot = true;	//Do it once
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//R
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	//G
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	//B
			  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	//Fan PWM
			  Set_RGB( 100, 0, 0 );
			  HAL_Delay(100);
			  Set_RGB(0, 100, 0);
			  HAL_Delay(100);
			  Set_RGB(0, 0, 100);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(En_Regulators_GPIO_Port, En_Regulators_Pin, GPIO_PIN_SET);	//Turn on Regulators
		  }


		  if(IsSystemCharging)
		  {
			  max17048_get_soc(&hi2c1, &CurrentBatteryPercentage);	//Get current Battery Percentage
			  if(CurrentBatteryPercentage < 75)
			  {
				  Set_RGB(100, 64, 0);
			  }
			  else if(CurrentBatteryPercentage >= 75 && CurrentBatteryPercentage < 90)
			  {
				  Set_RGB(0, 100, 0);
			  }else
			  {
				  Set_RGB(0, 0, 100);
			  }
		  }
		  else
		  {
			  Set_RGB(Remap(CurrentBatteryPercentage, 0, 100, 100, 0), 0, Remap(CurrentBatteryPercentage, 0, 100, 40, 100));
		  }

		  //Get Temperature
		  MeasuredTemperature = readThermistor();
		  /* Run PID controller with the new measured temperature */
		  PID_Compute(&Fan_PID);
		  /* Set New Duty cycle output*/
		  TIM1->CCR4 =PIDOut;



	  }
	  else
	  {
			/*
			* While system is Down,the code below will always run
			*/

			if(InitialSystemBoot)	//Start shutdown sequence
			{
				InitialSystemBoot = false;
				Set_RGB( 0, 100, 0 );
				HAL_Delay(200);
				Set_RGB(0, 0, 100);
				HAL_Delay(200);
				Set_RGB( 0, 100, 0 );
				HAL_Delay(200);
				Set_RGB(0, 0, 100);
				HAL_Delay(200);
				Set_RGB(0, 0, 0);
				HAL_GPIO_WritePin(En_Regulators_GPIO_Port, En_Regulators_Pin, GPIO_PIN_RESET);	//Turn off Regulators
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);	//Fan PWM
			}

			if(IsSystemCharging)
			{
				max17048_get_soc(&hi2c1, &CurrentBatteryPercentage);	//Get current Battery Percentage
				if(CurrentBatteryPercentage < 75)
				{
					Set_RGB(100, 64, 0);
				}
				else if(CurrentBatteryPercentage >= 75 && CurrentBatteryPercentage < 90)
				{
					Set_RGB(0, 100, 0);
				}else
				{
					Set_RGB(0, 0, 100);
				}
			}else
			{
				RequestSleep = true;
			}

	  }

	  if(RequestSleep)
	  {
		  EnterSleepModeWakeOnInturrupt();	//Will sleep here

		  RequestSleep = false;				//will wakeup here
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000004;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 4000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 50;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(En_Regulators_GPIO_Port, En_Regulators_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : En_Regulators_Pin */
  GPIO_InitStruct.Pin = En_Regulators_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(En_Regulators_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Power_Button_Pin */
  GPIO_InitStruct.Pin = Power_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Power_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_ALRT_Pin */
  GPIO_InitStruct.Pin = MAX_ALRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MAX_ALRT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BQ_INT_Pin */
  GPIO_InitStruct.Pin = BQ_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BQ_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
