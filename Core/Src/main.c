/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lsm6dso_reg.h"
#include "knowledge.h"
#include "NanoEdgeAI.h"
#include "max7219_Yncrea2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/************************************************************ NEAI algorithm defines begin ************************************************************/
/************************************************************ Global settings part ************************************************************/
#ifndef AXIS
  #define AXIS                          3                       /* Axis should be defined between 1 and 3 */
#endif
#ifndef SAMPLES
  #define SAMPLES                       128                     /* Should be between 16 & 4096 */
#endif
#define MAX_FIFO_SIZE                   256                     /* The maximum number of data we can get in the FIFO is 512 but here we define max to 256 for our need */
#define FIFO_FULL                       512                     /* FIFO full size */
#define FIFO_WORD                       7                       /* FIFO word size composed of 1 byte which is identification tag & 6 bytes of fixed data */
/************************************************************ Sensor type part ************************************************************/
#define ACCELEROMETER                                           /* Could be either ACCELEROMETER or GYROSCOPE */
/************************************************************ Sensors configuration part ************************************************************/
#ifdef ACCELEROMETER
  #ifndef ACCELEROMETER_ODR
    #define ACCELEROMETER_ODR           LSM6DSO_XL_ODR_417Hz   /* Shoud be between LSM6DSO_XL_ODR_12Hz5 and LSM6DSO_XL_ODR_6667Hz */
  #endif
  #ifndef ACCELEROMETER_FS
    #define ACCELEROMETER_FS            LSM6DSO_4g              /* Should be between LSM6DSO_2g and LSM6DSO_16g */
  #endif
#else
  #ifndef GYROSCOPE_ODR
    #define GYROSCOPE_ODR               LSM6DSO_GY_ODR_1667Hz   /* Shoud be between LSM6DSO_GY_ODR_12Hz5 and LSM6DSO_GY_ODR_6667Hz */
  #endif
  #ifndef GYROSCOPE_FS
    #define GYROSCOPE_FS                LSM6DSO_2000dps         /* Should be between LSM6DSO_125dps and LSM6DSO_2000dps */
  #endif
#endif
/************************************************************ Datalogger / NEAI mode part ************************************************************/
#ifndef NEAI_MODE
  #define NEAI_MODE                     1                       /* 0: Datalogger mode, 1: NEAI functions mode */
#endif
// #if (NEAI_MODE == 1)
//   #ifndef NEAI_LEARN_NB
//     #define NEAI_LEARN_NB               20                      /* Number of buffers to be learn by the NEAI library */
//   #endif
// #endif
#if (NEAI_MODE)
#ifndef NB_CLASSES
#define NB_CLASSES 3 /* Number of NEAI classes */
#endif
#endif
/************************************************************ NEAI algorithm defines end ************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static uint8_t whoamI, rst;
uint8_t neai_similarity = 0, neai_state = 0, first_comm = 1;
uint16_t sample_index = 0, id_class = 0;
volatile uint8_t drdy = 0;
uint16_t neai_cnt = 0, neai_buffer_ptr = 0, drdy_counter = 0;
float neai_time = 0.0;
float neai_buffer[AXIS * SAMPLES] = {0};
stmdev_ctx_t dev_ctx;
uint8_t previous_class = 0;  // Pour détecter les changements de mouvement
uint32_t last_movement_time = 0;  // Pour éviter les exécutions trop fréquentes
uint8_t debounce_flag = 0;

#if (NEAI_MODE)
static float class_output_buffer[NB_CLASSES];
const char *id2class[NB_CLASSES + 2] = {"unknown", "up-down", "forward-backward", "static", "circle"};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void lsm6dso_initialize(void);
static void lsm6dso_initialize_basics(void);
static void lsm6dso_initialize_fifo(void);
static void lsm6dso_get_buffer_from_fifo(uint16_t nb);
static float lsm6dso_convert_gyro_data_to_mdps(int16_t gyro_raw_data);
static float lsm6dso_convert_accel_data_to_mg(int16_t accel_raw_data);
static void iks01a3_i2c_stuck_quirk(void);

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
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  iks01a3_i2c_stuck_quirk();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_Init();
  lsm6dso_initialize();
  if (NEAI_MODE) {
    neai_state = neai_classification_init(knowledge);
    printf("Initialize NEAI library. NEAI init return: %d.\n",  neai_state);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (drdy) {
		  /* Reset data ready condition */
		  drdy = 0;
		  /* Read acceleration data */
		  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		  lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		  for (uint8_t i = 0; i < AXIS; i++) {
			  neai_buffer[(AXIS * drdy_counter) + i] = lsm6dso_convert_accel_data_to_mg(data_raw_acceleration[i]);
		  }
		  drdy_counter++;
		  if (drdy_counter >= SAMPLES) {
			  /* Set Output Data Rate */
			  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_OFF);
#if (NEAI_MODE)
			  neai_state = neai_classification(neai_buffer, class_output_buffer, &id_class);
			  printf("Class: %s. NEAI classification return: %d.\r\n", id2class[id_class], neai_state);
			  handleMovementActions();
#else
			  for (uint16_t i = 0; i < AXIS * SAMPLES; i++) {
				  printf("%.3f ", neai_buffer[i]);
			  }
			  printf("\r\n");
#endif
			  /* Reset drdy_counter in order to get a new buffer */
			  drdy_counter = 0;
			  /* Clean neai buffer */
			  memset(neai_buffer, 0x00, AXIS * SAMPLES * sizeof(float));
			  /* Set Output Data Rate */
			  lsm6dso_xl_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
		  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 200;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin
                           L4_Pin L5_Pin L6_Pin L7_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin|L5_Pin|L6_Pin|L7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BP1_increaseSpeed_Pin BP2_reduceSpeed_Pin */
  GPIO_InitStruct.Pin = BP1_increaseSpeed_Pin|BP2_reduceSpeed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_ACC_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_ACC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_ACC_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Redirecting stdout to USART2 which is connected on the STLINK port
  * @retval
  * @param
  */
int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart2, &*c, 1, 10);
 return ch;
}

/**
  * @brief  EXTI line rising detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
	  case GYRO_ACC_INT_Pin:
		  drdy = 1;
		  break;

	  case BP1_increaseSpeed_Pin:
		  if (!debounce_flag) {
			  debounce_flag = 1;
			  __HAL_TIM_SET_COUNTER(&htim6, 0);
			  HAL_TIM_Base_Start_IT(&htim6);
			  printf("Motor speed increased\r\n");
		  }

	  case BP2_reduceSpeed_Pin:
		  if (!debounce_flag) {
			  debounce_flag = 1;
			  __HAL_TIM_SET_COUNTER(&htim6, 0);
			  HAL_TIM_Base_Start_IT(&htim6);
			  printf("Motor speed reduced\r\n");
		  }
	}
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Initialize LSM6DSO sensor interface
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize()
{
  lsm6dso_initialize_basics();
#ifdef ACCELEROMETER
  /* Accelelerometer configuration */
  lsm6dso_xl_data_rate_set(&dev_ctx, ACCELEROMETER_ODR);
  lsm6dso_xl_full_scale_set(&dev_ctx, ACCELEROMETER_FS);
#else
  /* Gyroscope configuration */
  lsm6dso_gy_data_rate_set(&dev_ctx, GYROSCOPE_ODR);
  lsm6dso_gy_full_scale_set(&dev_ctx, GYROSCOPE_FS);
#endif
  lsm6dso_initialize_fifo();
}

/*
 * @brief  Initialize LSM6DSO basics
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize_basics()
{
  /* Check device ID */
  whoamI = 0;

  do {
    HAL_Delay(20);
    lsm6dso_device_id_get(&dev_ctx, &whoamI);
  } while(whoamI != LSM6DSO_ID);

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
}

/*
 * @brief  Initialize LSM6DSO internal FIFO
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_initialize_fifo()
{
#ifdef ACCELEROMETER
  /* Batch odr config */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, ACCELEROMETER_ODR);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, 0);
#else
  /* Batch odr config */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, 0);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, GYROSCOPE_ODR);
#endif
  /* FIFO MODE */
  lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_FIFO_MODE);
  /* Watermark config */
  if (SAMPLES <= MAX_FIFO_SIZE) {
    lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) SAMPLES);
  }
  else {
    lsm6dso_fifo_watermark_set(&dev_ctx, (uint16_t) MAX_FIFO_SIZE);
  }
  /* Need to enable interrupt pin when wtm is reached */
  uint8_t ctrl = 0x08;
  lsm6dso_write_reg(&dev_ctx, LSM6DSO_INT1_CTRL, (uint8_t *) &ctrl, 1);

  /* Configuration de l'interrupt DATA_READY sur INT1 */
  lsm6dso_pin_int1_route_t int1_route = {0};
  int1_route.drdy_xl = 1;  // Active l'interrupt data-ready pour l'accéléromètre
  lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);

  printf("Interrupt DATA_READY configuree sur INT1\n");
}

/*
 * @brief  Get accelerometer or gyroscope data from
 *         LSM6DSO using the internal FIFO buffer
 *
 * @param  No
 *
 * @return No
 *
 */
static void lsm6dso_get_buffer_from_fifo(uint16_t nb)
{
  uint8_t reg_tag = 0;
  uint8_t buff_tmp[nb * FIFO_WORD];
  /*
   * The data stored in FIFO are accessible from dedicated registers and each FIFO word is composed of 7
   * bytes: one tag byte (FIFO_DATA_OUT_TAG (78h)), in order to identify the sensor, and 6 bytes of fixed data
   * (FIFO_DATA_OUT registers from (79h) to (7Eh))
   * So, here we read the fifo in only one transaction in order to save time
   */
  lsm6dso_read_reg(&dev_ctx, LSM6DSO_FIFO_DATA_OUT_TAG, buff_tmp, nb * FIFO_WORD);
  for (uint16_t i = 0; i < nb; i++) {
    /* According to the datasheet, the TAG_SENSOR is the 5 MSB of the FIFO_DATA_OUT_TAG register, so we shift 3 bits to the right */
    reg_tag = buff_tmp[FIFO_WORD * i] >> 3;
    if(reg_tag == LSM6DSO_XL_NC_TAG) {
      for(uint8_t j = 0; j < AXIS; j++) {
	neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = lsm6dso_convert_accel_data_to_mg((uint16_t) buff_tmp[(FIFO_WORD * i) + (2 * j) + 2] << 8 | buff_tmp[(FIFO_WORD * i) + (2 * j) + 1]);
      }
    }
    else if(reg_tag == LSM6DSO_GYRO_NC_TAG) {
    for(uint8_t j = 0; j < AXIS; j++) {
      neai_buffer[(AXIS * neai_buffer_ptr) + (AXIS * i) + j] = lsm6dso_convert_gyro_data_to_mdps((uint16_t) buff_tmp[(FIFO_WORD * i) + (2 * j) + 2] << 8 | buff_tmp[(FIFO_WORD * i) + (2 * j) + 1]);
    }
    }
  }
  neai_buffer_ptr += nb;
  if (neai_buffer_ptr == SAMPLES) {
    neai_buffer_ptr = 0;
  }
}

/*
 * @brief  Convert gyroscope raw data to milli degrees per second (mdps)
 *
 * @param  gyro_raw_data: which is gyroscope raw data
 *                        depending on the full scale selected
 *
 * @return The converted value in milli degrees per second (mdps)
 *
 */
static float lsm6dso_convert_gyro_data_to_mdps(int16_t gyro_raw_data)
{
  float gyro_data_mdps = 0.0;
#ifdef GYROSCOPE
  switch (GYROSCOPE_FS)
  {
  case LSM6DSO_125dps:
    gyro_data_mdps = lsm6dso_from_fs125_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_250dps:
    gyro_data_mdps = lsm6dso_from_fs250_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_500dps:
    gyro_data_mdps = lsm6dso_from_fs500_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_1000dps:
    gyro_data_mdps = lsm6dso_from_fs1000_to_mdps(gyro_raw_data);
    break;
  case LSM6DSO_2000dps:
    gyro_data_mdps = lsm6dso_from_fs2000_to_mdps(gyro_raw_data);
    break;
  default:
    gyro_data_mdps = 0.0;
    break;
  }
#endif
  return gyro_data_mdps;
}

/*
 * @brief  Convert accelerometer raw data to milli-G' (mg)
 *
 * @param  accel_raw_data: which is accelerometer raw data
 *                        depending on the full scale selected
 *
 * @return The converted value in milli-G' (mg)
 *
 */
static float lsm6dso_convert_accel_data_to_mg(int16_t accel_raw_data)
{
  float accel_data_mg = 0.0;
#ifdef ACCELEROMETER
  switch (ACCELEROMETER_FS)
  {
  case LSM6DSO_2g:
    accel_data_mg = lsm6dso_from_fs2_to_mg(accel_raw_data);
    break;
  case LSM6DSO_4g:
    accel_data_mg = lsm6dso_from_fs4_to_mg(accel_raw_data);
    break;
  case LSM6DSO_8g:
    accel_data_mg = lsm6dso_from_fs8_to_mg(accel_raw_data);
    break;
  case LSM6DSO_16g:
    accel_data_mg = lsm6dso_from_fs16_to_mg(accel_raw_data);
    break;
  default:
    accel_data_mg = 0.0;
    break;
  }
#endif
  return accel_data_mg;
}

/*
 * Pressing the reset button while the sensor is answering to a read request
 * might lead to disaster.
 * In this case the device is stuck, waiting for pulses on SCL to finish the
 * previous transfer.
 * While stuck the sensor keep the SDA low.
 *
 * As a workaround we simply configure the SCL pin as a GPIO and send a burst
 * of pulses to bring the sensor back to an idle state.
 */
static void iks01a3_i2c_stuck_quirk(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure SCL as a GPIO */
  GPIO_InitStruct.Pin = SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

  /* Send a burst of pulses on SCL */
  int pulses = 20;
  do {
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
  } while (pulses--);

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_DISABLE();
}


void clearAllLEDs(void) {
    uint16_t L_Pin[8] = {L0_Pin, L1_Pin, L2_Pin, L3_Pin, L4_Pin, L5_Pin, L6_Pin, L7_Pin};
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(L0_GPIO_Port, L_Pin[i], GPIO_PIN_RESET);
    }
    MAX7219_Clear();
}


void staticSequence(void) {
	uint16_t L_Pin[8] = {L0_Pin, L1_Pin, L2_Pin, L3_Pin, L4_Pin, L5_Pin, L6_Pin, L7_Pin};

	MAX7219_DisplayChar(1, 'S');
	MAX7219_DisplayChar(2, 'T');
	MAX7219_DisplayChar(3, 'A');
	MAX7219_DisplayChar(4, 'T');
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(L0_GPIO_Port, L_Pin[i], GPIO_PIN_SET);
	}
}


void forwardBackwardSequence(void) {
	uint16_t L_Pin[8] = {L0_Pin, L1_Pin, L2_Pin, L3_Pin, L4_Pin, L5_Pin, L6_Pin, L7_Pin};

	MAX7219_DisplayChar(1, 'F');
	MAX7219_DisplayChar(2, 'O');
	MAX7219_DisplayChar(3, 'B');
	MAX7219_DisplayChar(4, 'A');

	for (int i = 0; i < 2; i++) {
		for (int j = 7; j > -1; j--) {
			HAL_GPIO_TogglePin(L0_GPIO_Port, L_Pin[j]);
			HAL_Delay(50);
		}
	}
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 8; j++) {
			HAL_GPIO_TogglePin(L0_GPIO_Port, L_Pin[j]);
			HAL_Delay(50);
		}
	}
}

/* Fonction principale de gestion des mouvements */
void handleMovementActions(void) {
    uint32_t current_time = HAL_GetTick();

    // Éviter les exécutions trop fréquentes (minimum 2 secondes entre les actions)
    if (current_time - last_movement_time < 2000) {
        return;
    }
	clearAllLEDs();  // Nettoyer l'affichage précédent
	HAL_Delay(100);

	// Exécuter l'action correspondant au mouvement détecté
	switch(id_class) {
		case 1: // "up-down"
			// upDownSequence();
			break;

		case 2: // "forward-backward"
			forwardBackwardSequence();
			break;

		case 3: // "static"
			staticSequence();
			break;

		case 4: // "circle"
			// circleSequence();
			break;

		case 0: // "unknown"
		default:
			// unknownSequence();
			break;
	}

	previous_class = id_class;
	last_movement_time = current_time;
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
