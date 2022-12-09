/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stm32f4xx_hal.h"
#include<stdio.h>
#include<math.h>
#include<string.h>
#include<stdint.h>

#include "ICM_20648.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define S_TIME  0.0005
#define CM_COUNT 4800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
//printf
int _write(int, char*, int);
//initializes variable
void variable_init();
//Full Color LED
void FullColor_LED(int, int, int, int);
//gyro_swicth
int Gyro_Switch();
//Secondary Run Section
void Run_Section();
//Analog_Value_Get
void Sensor_Get();
//Get the Max&Min sensor value and median of it
void Sensor_Update();
//calibrate Line Sensor
void Sensor_Calibration();
//Read the value of encoder and calculate the speed
void Encoder_Accel_Speed();
//Calculate Mileage
void Calculate_Mileage();
//speed control
void Speed_Control();
//Steering Motor Control
void Steering_Motor();
//Turning control
void Turning_Control();
//Motor rotation direction
void Motor_Setting();
//Start and Stop
void Side_Sens();
//Erase Flash Data
void eraseFlash(uint8_t sector);
//Write Data to Flash
void writeFlash(uint32_t address, uint8_t *data, uint32_t size, uint8_t sector);
//load Data from Flash
void loadFlash(uint32_t address, uint8_t *data, uint32_t size);
void *memcpy(
		void * restrict s1,
		const void * restrict s2,
		size_t n
);


//Flash Read&Write
const uint32_t start_address_sector9 = 0x80A0000;	//sector9 start address
const uint32_t end_address_sector9 = 0x80BFFFF;		//sector9 end address
const uint32_t start_address_sector10 = 0x80C0000;	//sector10 start address
const uint32_t end_address_sector10 = 0x80DFFFF;		//sector10 end address
const uint32_t start_address_sector11 = 0x80E0000;	//sector11 start address
const uint32_t end_address_sector11 = 0x80FFFFF;		//sector11 end address
//Flash Data
typedef struct
{
	//uint16_t speed_RL[4800];
	int16_t Omega[CM_COUNT];
	uint16_t mile_R[CM_COUNT];
	uint16_t mile_L[CM_COUNT];
}WRITE_DATA_SECTOR11;

typedef struct
{
	//uint16_t speed_RL[4800];
	int16_t Omega[CM_COUNT];
	uint16_t mile_R[CM_COUNT];
	uint16_t mile_L[CM_COUNT];
}READ_DATA_SECTOR11;

typedef struct
{
	double Sensor_R_Delta;
	double Sensor_L_Delta;
}WRITE_DATA_SECTOR9;
typedef struct
{
	double Sensor_R_Delta;
	double Sensor_L_Delta;
}READ_DATA_SECTOR9;

typedef struct
{
	uint16_t time[CM_COUNT];
	uint16_t milage[CM_COUNT];
	uint16_t marker_milage[200];
}WRITE_DATA_SECTOR10;
typedef struct
{
	uint16_t time[CM_COUNT];
	uint16_t milage[CM_COUNT];
	uint16_t marker_milage[200];
}READ_DATA_SECTOR10;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//ADC
uint16_t ADC_Sensor_Value[5];
int Stime = 0;
int St_Value[9], Sens_Right1[9], Sens_Right2[9], Sens_Left1[9], Sens_Left2[9];
int Steering_Value, Sensor_R, Sensor_L;
int Sensor_R_Min = 200, Sensor_R_Max = 200, Sensor_L_Min = 200, Sensor_L_Max = 200;
double Sensor_RC, Sensor_LC;
int Steering_Right_Value = 1255, Steering_Left_Value = 3000;

//Speed
double Speed, Speed_Ref = 1.6, Speed_R, Speed_L; //m/s
double Coefficient = 1.0045;
double SpeedR_Acc = 0, SpeedL_Acc = 0;

//IMU
uint16_t who_i_am;
double x_accel, y_accel, z_gyro, pre_zg = 0;

//mileage
double Mileage_R = 0, Mileage_L = 0, Mileage = 0, Score_Time = 0, Milage = 0;
int Write_Flag = 0, data_num = 0;
WRITE_DATA_SECTOR11 write_data_sector11 = {{0}};
READ_DATA_SECTOR11 read_data_sector11 = {{0}};
WRITE_DATA_SECTOR10 write_data_sector10 = {{0}};
READ_DATA_SECTOR10 read_data_sector10 = {{0}};
WRITE_DATA_SECTOR9 write_data_sector9 = {0};
READ_DATA_SECTOR9 read_data_sector9 = {0};
uint8_t Radius_Flag[CM_COUNT];
uint16_t Second_Mileage[CM_COUNT];
uint8_t Side_Right_Flag = 0;

//int Encoder_Right, Encoder_Left;
int countorperiod_R, countorperiod_L;
double theta, Turning_radius;
double i_speed_value_R = 0, i_speed_value_L = 0;

//Steering_buffer
double Steering_I_Buff = 0, Steering_D_Buff = 0, Steering_Buff = 0;
//Motor_duty
int Motor_CountorPeriod_R = 0, Motor_CountorPeriod_L = 0;
double Motor_Right = 0, Motor_Left = 0, Motor_Steering = 0;

//Flag
uint8_t Motor_Flag = 0;
//Start and Stop
uint8_t Stop_Flag = 0;
//Secondary Run
uint8_t Secondary = 0;
uint8_t Section_num = 0, Section_num_1 = 1;
uint16_t Section[400][2];
//uint8_t Section_radius[1000];
int SF = 0;
double Velocity;

//Encoder_Flag
//uint8_t encA_Flag = 0, encB_Flag = 1;
//int enc_cnt = 0;

double delta;

//Debug
double debug_value;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int gyro_switch = 0;
  uint8_t switch_break_flag = 0;
  int start_time = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Sensor_Value, 5);

  //PeriodElapsed
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim9);

  //Motor_PWM
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }

  //Encoder
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  FullColor_LED(1, 1, 1, 1);
	  FullColor_LED(2, 1, 1, 1);
	  //Speed_Ref = 1.7;
	  //printf("test\r\n");
	  //printf("X_Accel => %f	Y_Accel => %f	Z_Gyro => %f\r\n", x_accel, y_accel, z_gyro);
	  //printf("Encorder_L => %d	Encoder_R => %d\r\n", Encoder_Left, Encoder_Right);
	  //printf("Speed => %f\r\n", Speed);
	  //printf("theta => %f	ADC => %d\r\n", theta, Steering_Value);
	  printf("St => %d	R => %0.3f	L => %0.3f	S_L => %0.3f	S_R => %0.3f\r\n", Steering_Value, Sensor_RC, Sensor_LC, Speed_L, Speed_R);

	  gyro_switch = Gyro_Switch();
	  switch(gyro_switch)
	  {
	  case 1:
		  //A
		  FullColor_LED(1, 0, 0, 1);
		  Sensor_Calibration();
		  break;
	  case 2:
		  //B
		  FullColor_LED(1, 1, 0, 0);
		  Secondary = 0;
		  writeFlash(start_address_sector11, (uint8_t*)&write_data_sector11, sizeof(WRITE_DATA_SECTOR11), 11);
		  writeFlash(start_address_sector10, (uint8_t*)&write_data_sector10, sizeof(WRITE_DATA_SECTOR10), 10);
		  break;
	  case 3:
		  //C
		  FullColor_LED(1, 1, 1, 0);
		  Speed_Ref = 1.0;
		  break;
	  case 4:
		  //D
		  FullColor_LED(1, 0, 1, 0);
		  loadFlash(start_address_sector11, (uint8_t*)&read_data_sector11, sizeof(READ_DATA_SECTOR11));
		  loadFlash(start_address_sector10, (uint8_t*)&read_data_sector10, sizeof(READ_DATA_SECTOR10));
		  //for(int i = 0; i < 4800; i ++) printf("%d	%d	%d	%d\r\n", read_data_sector11.mile_L[i], read_data_sector11.mile_R[i], read_data_sector11.Omega[i], read_data_sector10.time[i]);
		  //for(int i = 0; i < 4800; i++) printf("%d\r\n", read_data_sector10.milage[i]);
		  //printf("Section_num => %d\r\n", Section_num);
		  Run_Section();
		  Secondary = 1;
		  break;
	  case 5:
		  //Center
		  FullColor_LED(1, 1, 0, 1);
		  //Speed_Ref = 1.6;
		  Mileage = Mileage_R = Mileage_L = Score_Time = SF = data_num = Milage = Side_Right_Flag = 0;
		  loadFlash(start_address_sector9, (uint8_t*)&read_data_sector9, sizeof(READ_DATA_SECTOR9));
		  Section_num = 0;
		  Section_num_1 = 1;
		  switch_break_flag = 1;
		  break;
	  default:
		  break;
	  }

	  if(switch_break_flag) HAL_Delay(2000);

	  while(switch_break_flag == 1)
	  {
		  Side_Sens();
		  Motor_Flag = 1;
		  FullColor_LED(1, 0, 0, 1);
		  //FullColor_LED(2, 0, 0, 1);

		  //printf("%f	%f\r\n", debug_value, Score_Time);
		  //printf("Duty => %lf	Delta => %lf\r\n", Motor_Steering, delta);
		  //printf("St => %d	R => %0.3f	L => %0.3f	S_L => %f	S_R => %f\r\n", Steering_Value, Sensor_RC, Sensor_LC, Speed_L, Speed_R);
		  //printf("%f\r\n", i_speed_value_R);
		  //printf("Speed_L => %f	Speed_R => %f\r\n", Speed_L, Speed_R);

		  if (Stop_Flag >= 2) break;
		  if (Gyro_Switch() == 3) break;
		  if (Sensor_RC + Sensor_LC >= 1800) break;
		  HAL_Delay(0.05);
	  }
	  if(Motor_Flag)
	  {
		  while(Speed_Ref >= 0.02)
		  {
			  Speed_Ref -= 0.015;
			  HAL_Delay(1);
		  }
	  }
	  start_time = 0;
	  switch_break_flag = 0;
	  variable_init();
	  HAL_Delay(100);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 839;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 420;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 209;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 839;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 209;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 39;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(icm_20648_CS_GPIO_Port, icm_20648_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA3 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB11 PB12
                           PB13 PB14 PB15 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : icm_20648_CS_Pin */
  GPIO_InitStruct.Pin = icm_20648_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(icm_20648_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//printf
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 0xFFFF);
	return len;
}

//initializes variable
void variable_init()
{
	Motor_Flag = 0;
	Stop_Flag = 0;
	i_speed_value_R = 0;
	i_speed_value_L = 0;
	who_i_am = IMU_init();
	//Score_Time = 0;
}

//Full Color LED
//arg1 LED1 or LED2, arg2~4 RGB
void FullColor_LED(int i, int r, int g, int b)
{
	if(i == 1)
	{
		if(r != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		if(g != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		if(b != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	}
	if(i == 2)
	{
		if(r != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		if(g != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		if(b != 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
}

//gyro_swicth
int Gyro_Switch()
{
	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);	//A
	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);	//C
	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);	//B
	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);	//Center
	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);	//D
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) return 1;
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0) return 3;
	else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) return 2;
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 0) return 5;
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 0) return 4;
	else return 0;
}

//Secondary Run Section
void Run_Section()
{
	double velocity, angular_velocity, curvature_radius, angular_velocity_enco;
	double coefficient = 1.0045, boundary = 0.41;
	int j = 1;
	for(int i = 0; i < CM_COUNT; i++)
	{
		//velocity = (double)write_data_sector11.speed_RL[i] / 1000;
		velocity = ((double)read_data_sector11.mile_L[i] + (double)read_data_sector11.mile_R[i]) / 2000;
		//angular_velocity = (double)write_data_sector11.Omega[i] / 100;
		angular_velocity_enco = ((double)read_data_sector11.mile_R[i] - (double)read_data_sector11.mile_L[i]) / (0.12 * 1000);
		angular_velocity = (angular_velocity_enco * 0.4) + ((double)write_data_sector11.Omega[i] / 100)*0.6;
		curvature_radius = velocity / angular_velocity;
		if(curvature_radius >= boundary || curvature_radius <= -boundary) Radius_Flag[i] = 1;
		else Radius_Flag[i] = 0;

		if(i > 0) Second_Mileage[i] = Second_Mileage[i-1] + velocity*(((double)read_data_sector10.time[i] - (double)read_data_sector10.time[i-1])/10);
		else Second_Mileage[0] = velocity * ((double)read_data_sector10.time[0] / 10);

		//printf("%f	%f\r\n", velocity, angular_velocity);
	}

	Section[0][0] = 0;
	for(int i = 0; i < CM_COUNT; i++)
	{
		if(Radius_Flag[i] && !Radius_Flag[i+1] && Radius_Flag[i+2]) i+=2;
		else if(Radius_Flag[i] && !Radius_Flag[i+1])
		{
			Section[j][0] = Second_Mileage[i];
			Section[j][1] = 1;
			j++;
		}
		else if(!Radius_Flag[i] && Radius_Flag[i+1])
		{
			Section[j][0] = Second_Mileage[i];
			Section[j][1] = 0;
			j++;
		}
	}
	HAL_Delay(500);

	for(int i = 0; i < j; i++)
	{
		if(!Section[i][1] && !Section[i+1][1])
		{
			for(int k = i; k < j; k++)
			{
				Section[k][1] = Section[k+1][1];
			}
			j--;
		}
	}
	/*
	for(int i = 0; i < j; i++)
	{
		printf("%d	%d\r\n", Section[i][0], Section[i][1]);
	}
	*/

	j = 1;
}

//Read the sensor value
void Sensor_Get()
{
	if (Stime >= 9) Stime = 0;
	St_Value[Stime] = ADC_Sensor_Value[0];
	Sens_Left2[Stime] = ADC_Sensor_Value[2];
	Sens_Left1[Stime] = ADC_Sensor_Value[1];
	Sens_Right1[Stime] = ADC_Sensor_Value[4];
	Sens_Right2[Stime] = ADC_Sensor_Value[3];
	Stime++;
}

//Get the Max&Min sensor value and median of it
void Sensor_Update()
{
	int i, j, SensBuf;
	for(i = 0; i < 9; i++)
	{
		for(j = 9; j > i; j--)
		{
			if(St_Value[j-1] > St_Value[j])
			{
				SensBuf = St_Value[j-1];
				St_Value[j-1] = St_Value[j];
				St_Value[j] = SensBuf;
			}
			if(Sens_Right1[j-1] > Sens_Right1[j])
			{
				SensBuf = Sens_Right1[j-1];
				Sens_Right1[j-1] = Sens_Right1[j];
				Sens_Right1[j] = SensBuf;
			}
			if(Sens_Right2[j-1] > Sens_Right2[j])
			{
				SensBuf = Sens_Right2[j-1];
				Sens_Right2[j-1] = Sens_Right2[j];
				Sens_Right2[j] = SensBuf;
			}
			if(Sens_Left1[j-1] > Sens_Left1[j])
			{
				SensBuf = Sens_Left1[j-1];
				Sens_Left1[j-1] = Sens_Left1[j];
				Sens_Left1[j] = SensBuf;
			}
			if(Sens_Left2[j-1] > Sens_Left2[j])
			{
				SensBuf = Sens_Left2[j-1];
				Sens_Left2[j-1] = Sens_Left2[j];
				Sens_Left2[j] = SensBuf;
			}
		}
	}

	Steering_Value = St_Value[5];
	Sensor_R = (Sens_Right1[5] + Sens_Right2[5]) / 2;
	Sensor_L = (Sens_Left1[5] + Sens_Left2[5]) / 2;
	//Sensor_R = Sens_Right1[5];
	//Sensor_L = Sens_Left1[5];

	Sensor_RC = (((double)Sensor_R - (double)Sensor_R_Min) / read_data_sector9.Sensor_R_Delta) * 1000;
	Sensor_LC = (((double)Sensor_L - (double)Sensor_L_Min) / read_data_sector9.Sensor_L_Delta) * 1000;
	Sensor_RC = Sensor_R;
	Sensor_LC = Sensor_L;
}

//calibrate Line Sensor
void Sensor_Calibration()
{
	if(Sensor_R <= Sensor_R_Min) Sensor_R_Min = Sensor_R;
	if(Sensor_R >= Sensor_R_Max) Sensor_R_Max = Sensor_R;
	if(Sensor_L <= Sensor_L_Min) Sensor_L_Min = Sensor_L;
	if(Sensor_L >= Sensor_L_Max) Sensor_L_Max = Sensor_L;

	write_data_sector9.Sensor_R_Delta = (double)Sensor_R_Max - (double)Sensor_R_Min;
	write_data_sector9.Sensor_L_Delta = (double)Sensor_L_Max - (double)Sensor_L_Min;

	writeFlash(start_address_sector9, (uint8_t*)&write_data_sector9, sizeof(WRITE_DATA_SECTOR9), 9);
	printf("%f\r\n", read_data_sector9.Sensor_R_Delta);
}

//Read the value of encoder and calculate the speed
void Encoder_Accel_Speed()
{
	int Encoder_Right, Encoder_Left;
	double omega, Speed_Acc;
	Encoder_Right = (TIM2 -> CNT) - 32767;
	Encoder_Left = (TIM3 -> CNT) - 32767;
	TIM2 -> CNT = 32767;
	TIM3 -> CNT = 32767;
	//read_accel_data();
	//read_gyro_data();
	//x_accel = ((float)xa / 2048) * 9.81;
	//y_accel = ((float)ya / 2048) + 98.1;
	//z_gyro = ((float)zg / 16.4) * (M_PI/180);
	//Speed = (((float)2970 * M_PI) / (float)11469) * ((Encoder_Right + Encoder_Left) / (float)2);
	//calculate the speed that is [m/s] on 07/01
	//Speed = (((float)1512 * M_PI) / (float)65536) * ((Encoder_Right + Encoder_Left) / (float)2);
	/*
	omega = z_gyro * 10;
	Speed_Acc += x_accel * S_TIME;
	SpeedR_Acc = Speed_Acc + 55*omega;	//+-OK
	SpeedL_Acc = Speed_Acc - 55*omega;
	*/
	//Speed_R = -(((double)3108 * M_PI) / (double)65534) * (double)Encoder_Right;
	Speed_R = (7. * 14.0 * M_PI * (double)Encoder_Right) / (40960. * 0.5);
	//Speed_L = (((double)3108 * M_PI) / (double)65534) * (double)Encoder_Left;
	Speed_L = (7. * 14.0 * M_PI * (double)Encoder_Left) / (40960. * 0.5);

	//Speed_R = SpeedR_Acc;
	//Speed_L = SpeedL_Acc;
	//Speed = (((Speed_R + Speed_L) / 2) + Speed_Acc) / 2;
	Speed = (Speed_R + Speed_L) / 2;
}

//Calculate Mileage
void Calculate_Mileage()
{
	double acc = 2, median = 0, percent, omega;
	if(Stop_Flag == 1)
	{
		//Mileage_R += Speed_R * S_TIME;
		//Mileage_L += Speed_L * S_TIME;
		Score_Time += S_TIME;
		read_gyro_data();
		z_gyro = zg;
		z_gyro = ((0.01)*(z_gyro) + (1.0 - (0.01))* (pre_zg)); // lowpath filter
		pre_zg = z_gyro;
		omega = (z_gyro /16.4) * M_PI / 180;

		//Mileage += (Speed_R + Speed_L) * (S_TIME);

		Milage += ((Speed_R*Coefficient + Speed_L)/2) * (S_TIME/10);
		if(data_num % 100) Side_Right_Flag = 1;

		if(!Secondary && Milage*100 >= data_num)
		{
			//if(Turning_radius < 0) radius = -Turning_radius;
			//else radius = Turning_radius;
			//write_data_sector11.speed_RL[data_num] = (Speed_R + Speed_L) * 500;
			write_data_sector11.Omega[data_num] = z_gyro * 1000;
			write_data_sector11.mile_R[data_num] = Speed_R*1000*Coefficient;
			write_data_sector11.mile_L[data_num] = Speed_L*1000;
			write_data_sector10.time[data_num] = Score_Time * 1000;
			write_data_sector10.milage[data_num] = Milage * 1000;
			data_num++;

			if(Section_num == Section_num_1)
			{
				write_data_sector10.marker_milage[Section_num] = Milage * 1000;
				Section_num_1++;
			}

		}
		else if(Secondary)
		{
			if(data_num == 0) data_num += 1;
			if(Section[data_num][0] <= (Milage*1000)) data_num++;

			if(Section_num == Section_num_1)
			{
				percent = Milage/read_data_sector10.marker_milage[Section_num];
				//if(percent <= 0.85) Section_num --;
				if(percent >= 1.2) Section_num ++;
				Milage = (read_data_sector10.marker_milage[Section_num] + 1) / 1000;
				Section_num_1++;
			}


			if(Section[data_num][1])
			{
				median = ((Section[data_num][0] - Section[data_num - 1][0]) / 2) + Section[data_num - 1][0];
				if(0.5*median <= Milage*1000)
				{
					Speed_Ref = Speed_Ref + acc*S_TIME;
					if(Speed_Ref >= 2.6) Speed_Ref = 2.6;
				}
				else if(0.5*median > Milage*1000)
				{
					Speed_Ref = Speed_Ref - acc*S_TIME;
					if(Speed_Ref <= 1.6) Speed_Ref = 1.6;
				}

				//Speed_Ref = 1.5;

				FullColor_LED(2, 0, 0, 1);
			}
			else if(!Section[data_num][1])
			{
				Speed_Ref = 1.6;
				FullColor_LED(2, 1, 0, 0);
			}
		}

		//if(Mileage >= 0.01*data_num) Write_Flag = 0;
	}
}

//speed control
void Speed_Control()
{
	double PGain_R = 190, IGain_R = 10, PGain_L = 190, IGain_L = 10;
	double delta_speed_R = (Motor_Right * 10) - Speed_R;
	double delta_speed_L = (Motor_Left * 10) - Speed_L;
	double delta_speed_RL = (((Motor_Right * 10) + (Motor_Left * 10))/2) - ((Speed_R + Speed_L)/2);
	i_speed_value_R += delta_speed_RL * S_TIME;
	if (i_speed_value_R >= 1000000) i_speed_value_R = 1000000;
	if (i_speed_value_R <= -1000000) i_speed_value_R = -1000000;
	//i_speed_value_L += delta_speed_L * S_TIME;
	if (i_speed_value_L >= 10000000) i_speed_value_L = 10000000;
	if (i_speed_value_L <= -10000000) i_speed_value_L = -10000000;
	Motor_CountorPeriod_R = PGain_R * delta_speed_R/* + IGain_R * i_speed_value_R*/;
	Motor_CountorPeriod_L = PGain_L * delta_speed_L/* + IGain_L * i_speed_value_R*/;

	//Motor_CountorPeriod = 0;
}

//Steering Motor Control
void Steering_Motor()
{
	double PGain = 0.26, DGain = 0.0009;
	delta = Sensor_RC - Sensor_LC;
	//double delta = (double)Sensor_R - (double)Sensor_L;
	Steering_I_Buff += delta * S_TIME;
	if(Steering_I_Buff >= 100000000) Steering_I_Buff = 100000000;
	if(Steering_I_Buff <= -100000000) Steering_I_Buff = -100000000;
	Steering_D_Buff = (Steering_Buff - delta) * S_TIME;
	Steering_Buff = delta;

	if(Steering_Value >= Steering_Right_Value && Steering_Value <= Steering_Left_Value && Motor_Flag)
	{
		//FullColor_LED(2, 0, 0, 1);
		//Motor_Steering = (delta * PGain) + (Steering_I_Buff * IGain) - (Steering_D_Buff * DGain);
		Motor_Steering = (delta * PGain) - (Steering_D_Buff * DGain);
	}
	else
	{
		FullColor_LED(2, 1, 0, 0);
		Motor_Steering = 0;
	}

	if (Motor_Steering >= 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Motor_Steering);
	}
	else if (Motor_Steering < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -Motor_Steering);
	}
}

//Turning control
void Turning_Control()
{
	double speed = 0, L = 55, deg = 0;
	if(Stop_Flag == 0) speed = 1.0;
	else speed = Speed_Ref;
	theta = M_PI * ((2100 - (double)Steering_Value) / 4096);
	Turning_radius = 40.1 / tan(theta);
	//if(theta >= -0.07 && theta <= 0.07) L = 10;
	//else L = 55;
	if(theta < 0) deg = (-theta * 180) / M_PI;
	else deg = (theta * 180) / M_PI;

	L = (pow(3, deg)) + pow(speed, 2) - 1;
	if(L >= 55) L = 55;
	if(Turning_radius <= 120) L = 75;
	Motor_Right = (Turning_radius - L) * (speed / Turning_radius);
	Motor_Left = (Turning_radius + L) * (speed / Turning_radius);
	//Motor_Right = Motor_Left = Motor_CountorPeriod;
}

//Motor rotation direction
void Motor_Setting()
{
	//int countorperiod_R, countorperiod_L;
	if(Motor_Flag == 0) countorperiod_R = countorperiod_L = 0;
	else if(Motor_Flag ==1)
	{
		//Right
		if (Motor_CountorPeriod_R >= 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			countorperiod_R = Motor_CountorPeriod_R;
		}
		else if (Motor_CountorPeriod_R < 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			countorperiod_R = -Motor_CountorPeriod_R;
		}

		//Left
		if (Motor_CountorPeriod_L >= 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			countorperiod_L = Motor_CountorPeriod_L;
		}
		else if (Motor_CountorPeriod_L < 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			countorperiod_L = -Motor_CountorPeriod_L;
		}
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, countorperiod_R);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, countorperiod_L);
}

void Side_Sens()
{
	double m_milage;
	if(Motor_Flag == 1)
	{
		int flag = 0, flag_L = 0;
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != 0)//L_Low
		{
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) //R_High
			{
				while(1)
				{
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) != 0) //R_Low
					{
						if(flag_L == 1)
						{
							flag = 0;
							//Section_num--;
						}
						else if(flag_L == 0) flag = 1;
						break;
					}
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) //L_High
					{
						flag_L = 1;
						flag = 0; 	//L_High
						if(Side_Right_Flag)
						{
							Section_num++;
							Side_Right_Flag = 0;
						}
						//m_milage = Milage;
					}
					else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != 0)
					{
						flag = 1;										//L_Low
					}
					//if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) != 0) break;
				}
				if(flag == 1) Stop_Flag += 1;
			}
			flag_L = 0;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)//L_High
		{
			while(1)
			{
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) != 0)
				{
					if(Side_Right_Flag)
					{
						Section_num++;
						Side_Right_Flag = 0;
					}
					//m_milage = Milage;
					break;
				}
			}
		}
		flag = 0;

		//write_data_sector10.marker_milage[Section_num] = m_milage * 1000;
	}
}

//Erase Flash Data
void eraseFlash(uint8_t sector)
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	if(sector == 9) erase.Sector = FLASH_SECTOR_9;
	else if(sector == 10) erase.Sector = FLASH_SECTOR_10;
	else if(sector == 11) erase.Sector = FLASH_SECTOR_11;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);
}

//Write Data to Flash
void writeFlash(uint32_t address, uint8_t *data, uint32_t size, uint8_t sector)
{
	HAL_FLASH_Unlock();
	eraseFlash(sector);

	for(uint32_t add = address; add < (address + size); add++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, add, *data);
		data++;
	}

	HAL_FLASH_Lock();
}

//load Data from Flash
void loadFlash(uint32_t address, uint8_t *data, uint32_t size)
{
	memcpy(data, (uint8_t*)address, size);
}

/*
void Encoder_A()
{
	if (!encA_Flag) encA_Flag = 1;
	else encA_Flag = 0;

	if (encA_Flag && !encB_Flag) enc_cnt++;
	else enc_cnt--;
}

void Encoder_B()
{
	if (!encB_Flag) encB_Flag = 1;
	else encB_Flag = 0;

	if (!encA_Flag && encB_Flag) enc_cnt--;
	else enc_cnt++;
}
*/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
    Encoder_A();
  if (GPIO_Pin == GPIO_PIN_1)
    Encoder_B();
}
*/
//Timer_PeriodElapsed_Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM9)
	{
		Sensor_Get();
	}
	//TIM5
	if (htim->Instance == TIM5)
	{
		Sensor_Update();
		Encoder_Accel_Speed();
		Steering_Motor();
		Turning_Control();
		Speed_Control();
		Motor_Setting();
		Calculate_Mileage();
	}
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
