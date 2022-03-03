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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <PID.h>
#include "AHRS.h"
#include "Altimeter\Detucted_Drivers\Motor.h"
#include "Altimeter\Detucted_Drivers\map.h"
#include <Screen.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//enum to define status of the program for handling interrupts
typedef enum {
	STARTING_STATUS, // in this state program isn't ready: all interrupt should be disabled
	MPU6050_INITIALIZED_STATUS, //in this state MPU has been configured
	MOTORS_ARMED_STATUS, //in this state motors are armed
	MOTORS_CONTROLLING_STATUS, //in this state motors control is running
	PAUSED_STATUS // in this state motors control is paused
} Program_Status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x300400c0
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x300400c0))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*variables needed to switch on and off the motors.*/
Program_Status program_status = STARTING_STATUS;
int time_5ms_increase = 0;

/* Create PID structure used for PID properties */
PID_config z_axis_PID;
PID_config Yaw_PID;
PID_config Pitch_PID;
PID_config Roll_PID;

/* Used to store value of altitude, yaw, pitch, roll need to be reached */
float desiredPitch = 0;
float desiredPitch = 0;
float desiredRoll = 0;

/* Time in seconds every which PID control is made */
const float dt = 0.05;

/*Structure used by IMU for initialization*/
AHRS_out ahrs;

//int enable_interrupt = 0; //variabile che utilizziamo per l'avvio delle funzioni successive al setup.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Callback_5ms(AHRS_out *ahrs);
void Callback_50ms();
void Callback_100ms();
void Setup_Motor_PID();
float* SpeedCompute(float virtualInputs[], float b, float l, float d);
//function to print on screen the status of the program
void print_IMU_init();
void print_mag_init();
void print_motor_arm();
void print_motor_start();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
//	screen_setup();
	print_IMU_init();
	MPU6050_Init();

	print_mag_init();
	MPU6050_Mag_Init(&ahrs.mag);

	//setting that MPU6050 is ready
	program_status = MPU6050_INITIALIZED_STATUS;
	print_motor_arm();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//Program is now handled by interrupt:HAL_GPIO_EXTI_Callback and HAL_TIM_PeriodElapsedCallback
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x00B03FDB;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 2400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 240-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void Setup_Motor_PID() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	/* Send arm signal to motors */
	Motor_Arm_All();
	HAL_Delay(2000);

	// Initialize PID structures used for PID properties
	// with their respective coefficents for proportional,
	// derivative and integrative
	PID_Init(&z_axis_PID, 0.7, 0.05, 0.3, dt, 0, 1);

	/*IMUs PID: they need to be tuned properly, right now Kp and Kd are set to 1*/
	PID_Init(&Pitch_PID, 1, 1, 0, dt, 0, 1);
	PID_Init(&Roll_PID, 1, 1, 0, dt, 0, 1);
	PID_Init(&Yaw_PID, 1, 1, 0, dt, 0, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (program_status == MOTORS_CONTROLLING_STATUS) {
		if (htim == &htim2) {
			Callback_5ms(&ahrs);
		}
		if (htim == &htim1) {
			Callback_50ms();
		}
		if (htim == &htim4) {
			Callback_100ms();
		}

	}
}

//interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//callback for B1 interrupt: do different operations based on the status of the program
	//by default do nothing meaning the screen and MPU6050 are configurated
	if (GPIO_Pin == GPIO_PIN_13) {
		switch (program_status) {
		//if MPU6050 is initializied motors are ready do be armed
		case MPU6050_INITIALIZED_STATUS:
			Setup_Motor_PID();
			program_status = MOTORS_ARMED_STATUS;
			print_motor_start();
			break;
		//if motors are armed program is enabled to start them
		case MOTORS_ARMED_STATUS:
			program_status = MOTORS_CONTROLLING_STATUS;
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_TIM_Base_Start_IT(&htim4);
			break;
		//if motors are running B1 stop them
		case MOTORS_CONTROLLING_STATUS:
//			HAL_TIM_Base_Stop_IT(&htim2);
//			HAL_TIM_Base_Stop_IT(&htim1);
//			HAL_TIM_Base_Stop_IT(&htim4);
			program_status = PAUSED_STATUS;
			Motors_Off();
			time_5ms_increase = 0;
			break;
		//if motors are paused B1 start them
		case PAUSED_STATUS:
			Motor_Arm_All();
			HAL_Delay(2000);
			program_status = MOTORS_CONTROLLING_STATUS;
			//HAL_TIM_Base_Start_IT(&htim2);
			//HAL_TIM_Base_Start_IT(&htim1);
			//HAL_TIM_Base_Start_IT(&htim4);
			break;
		//by default do nothing
		default:
			break;
		}
	}
}

void Callback_5ms(AHRS_out *ahrs) {
	MPU6050_Read_Accel(&ahrs->raw, &ahrs->temp);
	MPU6050_Read_Gir(&ahrs->raw, &ahrs->temp);
//	mag_read(&ahrs->mag);
	getYPR(&ahrs->mag, &ahrs->temp, &ahrs->ahrs_data);

	//Read_Acc(ahrs)---> N:B. da radianti per secondo a gradi per secondo
	//Read_Gyr(ahrs)---> N:B. da radianti a angoli
}

void Callback_50ms() {

	//Dati IMU che definiscono lo stato corrente del drone nei tre assi

	float Current_pitch = ahrs.ahrs_data.PitchDeg;
	float Current_roll = ahrs.ahrs_data.RollDeg;
	float Current_yaw = ahrs.ahrs_data.YawDeg;

	//Risultati PID che memorizziamo all'interno di un vettore di appoggio, virtualInputs

	float virtualInputs[4];

	virtualInputs[1] = PID_Compute(Current_pitch, desiredPitch, &Pitch_PID);
	virtualInputs[2] = PID_Compute(Current_roll, desiredRoll, &Roll_PID);
	virtualInputs[3] = PID_Compute(Current_yaw, desiredPitch, &Yaw_PID);

	float *Speeds;

	//computes motor speeds (B_4 is for 4-cell battery, if you use a 3-cell, change it with B_3)
	Speeds = SpeedCompute(virtualInputs, B_4, L, D);

	//converts the speed in a measure that can be read by the motors
	float motor1 = map(*(Speeds + 0), 0, MOTOR_MAX_SPEED_4, MOTOR_MIN_UP,
	MOTOR_MAX_UP);
	float motor2 = map(*(Speeds + 1), 0, MOTOR_MAX_SPEED_4, MOTOR_MIN_UP,
	MOTOR_MAX_UP);
	float motor3 = map(*(Speeds + 2), 0, MOTOR_MAX_SPEED_4, MOTOR_MIN_UP,
	MOTOR_MAX_UP);
	float motor4 = map(*(Speeds + 3), 0, MOTOR_MAX_SPEED_4, MOTOR_MIN_UP,
	MOTOR_MAX_UP);

	//writes the measure obtained in each PWM
	Motor_Write_up(MOTOR_1, motor1);
	Motor_Write_up(MOTOR_2, motor2);
	Motor_Write_up(MOTOR_3, motor3);
	Motor_Write_up(MOTOR_4, motor4);

}

void Callback_100ms() {
	//print_acc(ahrs.temp.accRoll, ahrs.temp.accPitch, ahrs.temp.accYaw);
	print_acc(ahrs.ahrs_data.RollDeg, ahrs.ahrs_data.PitchDeg,
			ahrs.ahrs_data.YawDeg);
//	print_acc(ahrs.ahrs_data.omegaRollDeg, ahrs.ahrs_data.omegaPitchDeg, ahrs.ahrs_data.omegaYawDeg);
	/*
	 */

}

/******************************************************************
 *Function name: SpeedCompute
 *Description  : Computes the speed that each motor has to generate to reach the desired attitude
 *Arguments    :
 * @param virtualInputs
 * @param b thrust coefficient
 * @param l distance between motor and center of the drone
 * @param d drag coefficient
 * @return array of the speeds
 */float* SpeedCompute(float virtualInputs[], float b, float l, float d) {

	static float Speeds_quad[4];
	static float Speeds[4];

	Speeds_quad[0] = (1 / (4 * b)) * virtualInputs[0]
			- (1 / (2 * l * b)) * virtualInputs[2]
			- (1 / (4 * d)) * virtualInputs[3];
	Speeds_quad[1] = (1 / (4 * b)) * virtualInputs[0]
			- (1 / (2 * l * b)) * virtualInputs[1]
			+ (1 / (4 * d)) * virtualInputs[3];
	Speeds_quad[2] = (1 / (4 * b)) * virtualInputs[0]
			+ (1 / (2 * l * b)) * virtualInputs[2]
			- (1 / (4 * d)) * virtualInputs[3];
	Speeds_quad[3] = (1 / (4 * b)) * virtualInputs[0]
			+ (1 / (2 * l * b)) * virtualInputs[1]
			+ (1 / (4 * d)) * virtualInputs[3];

	/*every speed needs to be mapped in a positive range [0, max speed^2], because the mathematical model
	 *and the control system consider the possibility to invert the rotation, resulting in negative speeds.
	 *This is out the actual possibilities of our physical system because we cannot invert rotation nor
	 *calculate a square root of a negative number.*/

	const float abs_speedQuad_MAX = 1 / (4 * b) + 1 / (2 * l * b) + 1 / (4 * d); //maximum reachable squared speed

	float speedQuad_min = -(1 / (2 * l * b)) - (1 / (4 * d)); //lower bound for squared speed 0.
	float speedQuad_Max = 1 / (4 * b); //upper bound for squared speed 0.
	Speeds[0] = sqrt(
			map(Speeds_quad[0], speedQuad_min, speedQuad_Max, 0,
					abs_speedQuad_MAX));

	speedQuad_min = -(1 / (2 * l * b)); //lower bound for squared speed 1.
	speedQuad_Max = 1 / (4 * b) + 1 / (4 * d); //upper bound for squared speed 1.
	Speeds[1] = sqrt(
			map(Speeds_quad[1], speedQuad_min, speedQuad_Max, 0,
					abs_speedQuad_MAX));

	speedQuad_min = -(1 / (4 * d)); //lower bound for squared speed 2.
	speedQuad_Max = 1 / (4 * b) + 1 / (2 * l * b); //upper bound for squared speed 2.
	Speeds[2] = sqrt(
			map(Speeds_quad[2], speedQuad_min, speedQuad_Max, 0,
					abs_speedQuad_MAX));

	//Speeds_quad[3] can't be negative so we just need to make the square root
	Speeds[3] = sqrt(Speeds_quad[3]);

	return Speeds;

}
void print_IMU_init() {
	memset(buffer1, 0, sizeof(buffer1));
	memset(buffer2, 0, sizeof(buffer2));
	memset(buffer3, 0, sizeof(buffer3));
	sprintf(buffer1, "inizializzazione");
	sprintf(buffer2, "IMU");
	sprintf(buffer3, "...");
	SSD1306_GotoXY(10, 10);
	SSD1306_Puts((char*) buffer1, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 20);
	SSD1306_Puts((char*) buffer2, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 30);
	SSD1306_Puts((char*) buffer3, &Font_7x10, 1);
	SSD1306_UpdateScreen();

}

void print_mag_init() {
	memset(buffer1, 0, sizeof(buffer1));
	memset(buffer2, 0, sizeof(buffer2));
	memset(buffer3, 0, sizeof(buffer3));
	sprintf(buffer1, "inizializzazione");
	sprintf(buffer2, "magnetometro");
	sprintf(buffer3, "...");
	SSD1306_GotoXY(10, 10);
	SSD1306_Puts((char*) buffer1, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 20);
	SSD1306_Puts((char*) buffer2, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 30);
	SSD1306_Puts((char*) buffer3, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}
void print_motor_arm() {
	sprintf(buffer1, "Premere B1");
	sprintf(buffer2, "per armare");
	sprintf(buffer3, "i motori...");
	SSD1306_GotoXY(10, 10);
	SSD1306_Puts((char*) buffer1, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 20);
	SSD1306_Puts((char*) buffer2, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 30);
	SSD1306_Puts((char*) buffer3, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}
void print_motor_start() {
	sprintf(buffer1, "Premere B1");
	sprintf(buffer2, "per avviare");
	sprintf(buffer3, "i motori...");
	SSD1306_GotoXY(10, 10);
	SSD1306_Puts((char*) buffer1, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 20);
	SSD1306_Puts((char*) buffer2, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	SSD1306_GotoXY(10, 30);
	SSD1306_Puts((char*) buffer3, &Font_7x10, 1);
	SSD1306_UpdateScreen();
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
	while (1) {
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

