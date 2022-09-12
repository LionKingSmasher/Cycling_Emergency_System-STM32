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

#include "lsm9ds1_reg.h"
#include "madgwick.h"
#include "rectrix_sms.h"
#include "hack_led.h"

#include <string.h>
#include <math.h>
//#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	void* hbus;
	uint8_t i2c_address;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} sensbus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
#define BOOT_TIME 20

#define    WAIT_TIME_MAG        60 //ms
#define    WAIT_TIME_XL        200 //ms
#define    WAIT_TIME_GY        800 //ms

#define    SAMPLES               5 //number of samples

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static sensbus_t imu_bus = {
		&SENSOR_BUS,
		LSM9DS1_IMU_I2C_ADD_H,
		0,
		0
};

static sensbus_t mag_bus = {
		&SENSOR_BUS,
		LSM9DS1_MAG_I2C_ADD_H,
		0,
		0
};

uint8_t rst;

hack_led_t led;

static int16_t data_raw_acceleration[3], st_data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3], st_data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3], st_data_raw_magnetic_field[3];
//static float val_offset[3];
static float acceleration_mg[3], st_acceleration_mg[3];
static float angular_rate_mdps[3], st_angular_rate_mdps[3];
static float magnetic_field_mgauss[3], st_magnetic_field_mgauss[3];
//static float cal_accel[3], cal_gyro[3], cal_mag[3];

static int16_t accelIntegral[3];
static int16_t accelGravityIntegral[3];
static int16_t gyroSmoothend[3];

static float aa_rot[3], last_aa[3];
static int16_t aaReal[3];
static float gy_rot[3];
static float ypr[3], ypr_rot[3];

static float acc_vsm;
static float w_vsm;
static float gravity_vsm;
static float sum_vsm;

static float yaw, roll, pitch;

static char chk_falling;

//const int COMMON_GRAVITY_Z = 1000;
float THRESHOLD_INT_ACCEL_X = 69.54 * 100;
float THRESHOLD_INT_ACCEL_Y = 121.80 * 100;
float THRESHOLD_INT_ACCEL_Z = 88.26 * 100;
float THRESHOLD_INT_GRAVITY_X = 79.06 * 100;
float THRESHOLD_INT_GRAVITY_Y = 134.78 * 100;
//int THRESHOLD_INT_GRAVITY_Z = 1;
float THRESHOLD_SMOOTH_GYRO_X = 41.40 * 2;
float THRESHOLD_SMOOTH_GYRO_Y = 49.86 * 2;
float THRESHOLD_SMOOTH_GYRO_Z = 111.76 * 2;
//float THRESHOLD_YAW = 360;
float THRESHOLD_PITCH = 280.98 * 45 / 100 / 2;
float THRESHOLD_ROLL = 349.16 * 50 / 100 / 2;
float THRESHOLD_ACCEL_DELTA = 39.30 * 100;


const float rotateZAngle = 1.0;

const float MOVING_AVERAGE_DECLINE = 0.1f;

extern float beta;
extern float grav[3];

float continuous_rating = 0.0f;
float normalized_continuous_rating = 0.0f;

const float GYRO_EXP_DECLINE_FACTOR = 0.1f;

double former_x = 0.0f, former_y = 0.0f, former_z = 0.0f;

//static const float min_st_mag_limit[] = {1000.0f, 1000.0f,  100.0f};
//static const float max_st_mag_limit[] = {3000.0f, 3000.0f, 1000.0f};
//
///* Self test limits in mg @ 2g*/
//static const float min_st_xl_limit[] = {70.0f, 70.0f,  70.0f};
//static const float max_st_xl_limit[] = {1500.0f, 1500.0f, 1500.0f};
//
///* Self test limits in mdps @ 2000 dps*/
//static const float min_st_gy_limit[] = {200000.0f, 200000.0f, 200000.0f};
//static const float max_st_gy_limit[] = {800000.0f, 800000.0f, 800000.0f};

char str[256];
rectirx_sms_t sms;

extern char count;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void lsm9ds1_self_test(void);
static int32_t platform_write_imu(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
static int32_t platform_read_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
static int32_t platform_write_mag(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
static int32_t platform_read_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
static void platform_delay(uint32_t ms);
double sigmoid_function(double a);
double norm(double a, double b, double c);
double evalContinuous(float yaw, float pitch, float roll,
					  float gx, float gy, float gz,
					  float ax, float ay, float az,
					  float* gravity);
void prepareData();
void rotateZ(float radAngle, float* point, float* rotatedPoint);
_Bool checkThreshold(float liveData, float thresholdVal);
char chk[11];

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
	HAL_StatusTypeDef ret;
	uint8_t buf[32];
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  buf[0] = 0xA0 | 0x12;

  sms.my_device = huart2;
  sms.sms_device = huart1;

  rectrix_init(&sms);
  setting_phone_number(&sms, "1111111111"); // Setting Phone Number

  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  init_hack_led(&led, GPIOB, GPIO_PIN_4);

  lsm9ds1_self_test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
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
  hi2c1.Init.Timing = 0x007026BD;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void lsm9ds1_self_test(void){
	double crashPropability;

	stmdev_ctx_t dev_ctx_imu;
	stmdev_ctx_t dev_ctx_mag;

	uint16_t AD_RES = 0;

	lsm9ds1_status_t reg;
	lsm9ds1_id_t whoamI;

	int16_t data_raw[3];

	dev_ctx_imu.write_reg = platform_write_imu;
	dev_ctx_imu.read_reg = platform_read_imu;
	dev_ctx_imu.handle = (void*)&imu_bus;

	dev_ctx_mag.write_reg = platform_write_mag;
	dev_ctx_mag.read_reg = platform_read_mag;
	dev_ctx_mag.handle = (void*)&mag_bus;

	platform_delay(BOOT_TIME);
	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

//	sprintf(str, "IMU: 0x%02x\r\nMAG:0x%02x\r\n", whoamI.imu, whoamI.mag);
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	if(whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
		rectrix_SendMessage(&sms, "Cycling Emergency System Disabled");
		while(1);
	}
	else{
		rectrix_SendMessage(&sms, "Cycling Emergency System Enabled")
	}

	lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

	do {
		lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
	}while(rst);

//	sprintf(str, "Reset Complete!\r\n");
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

//	sprintf(str, "Block Update Enable Complete!\r\n");
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
	lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

	lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_50Hz);
	lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);

//	sprintf(str, "Acceleometer filtering Complete!\r\n");
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
	lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

//	sprintf(str, "Gyrormeter filtering Complete!\r\n");
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_119Hz_LP);
	lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_LP_80Hz);

	unsigned int tick_start = 0, tick_end = 0,
				 emergency_tick_now = 0, emergency_tick_before = 0,
				 brightness_tick_before = 0;
	GPIO_PinState brightness_state = GPIO_PIN_SET;

	_Bool sms_trigger = 0u;

	while(1){
		double delta_x, delta_y, delta_z;
		tick_start = HAL_GetTick();

		if(tick_start - brightness_tick_before > 50 && count > 0) {
			hack_led_gpio_set_reset(&led, brightness_state);
			brightness_state = !brightness_state;
			brightness_tick_before = tick_start;
			count--;
		}

		if(tick_start - tick_end > (1000 * invSampleFreq2)){
			lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

			if ( reg.status_imu.xlda && reg.status_imu.gda ) {
			  /* Read imu data */
			  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			  lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
										   data_raw_acceleration);
			  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
										   data_raw_angular_rate);
			  for(int i = 0; i < 3; i++){
				acceleration_mg[i] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[i]) / 1000;
				angular_rate_mdps[i] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[i]) / 1000;
			  }

			}

			if ( reg.status_mag.zyxda ) {
			  /* Read magnetometer data */
			  memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
			  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
			  magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
										   data_raw_magnetic_field[0]);
			  magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
										   data_raw_magnetic_field[1]);
			  magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
										   data_raw_magnetic_field[2]);
	//		  sprintf(str, "[G] %4.2f\t%4.2f\t%4.2f\r\n",
	//				  magnetic_field_mgauss[0]/10, magnetic_field_mgauss[1]/10,
	//				  magnetic_field_mgauss[2]/10);
	//		  HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
			}



			madgwickUpdate(angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
						   acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
						   magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2],
						   invSampleFreq2);

			getGravity();
			getLinearAccel(aaReal, data_raw_acceleration);

//			computeAngles();

			pitch = getPitch();
			roll = getRoll();
			yaw = getYaw();

			ypr[0] = yaw;
			ypr[1] = pitch;
			ypr[2] = roll;


//			getLinearAccel(angular_rate_mdps, aaReal);

			prepareData();

			crashPropability = evalContinuous(yaw, pitch, roll,
											  angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
											  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
											  grav);

			sprintf(str, "%.2f\r\n", crashPropability);
							HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

			if(crashPropability > 0.5f && tick_start - emergency_tick_before > 1000){
				rectrix_SendMessage(&sms, "Helmet detected a Cycling accident!");
				emergency_tick_before = tick_start;
			}

			// Start ADC Conversion
			HAL_ADC_Start(&hadc1);
		   // Poll ADC1 Perihperal & TimeOut = 1mSec
			HAL_ADC_PollForConversion(&hadc1, (1000 * invSampleFreq2));
		   // Read The ADC Conversion Result & Map It To PWM DutyCycle
			AD_RES = HAL_ADC_GetValue(&hadc1);
			TIM2->CCR1 = (AD_RES<<4);

			sprintf(str,
					  "%d\r\n", AD_RES);
			HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

			if(count == 0){
				if(AD_RES < 2900){
					hack_led_set(&led, TURN_ON_1);
				}
				else{
					hack_led_off(&led);
				}
			}

			tick_end = tick_start;
		}
	}
}

static int32_t platform_write_imu(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len){
	sensbus_t *sensbus = (sensbus_t*)handle;

	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
}

int32_t platform_write_mag(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len){
	sensbus_t *sensbus = (sensbus_t*)handle;

	reg |= 0x80;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
}

static int32_t platform_read_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len){
	sensbus_t *sensbus = (sensbus_t*)handle;

	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
}

static int32_t platform_read_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len){
	sensbus_t *sensbus = (sensbus_t*)handle;

	reg |= 0x80;
	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
}

static void platform_delay(uint32_t ms){
	HAL_Delay(ms);
}

double sigmoid_function(double x){
	return x / (2 * (1 + abs(x))) + 0.5;
}

double norm(double a, double b, double c){
#define sq(X) pow(X, 2)
	return sqrt(sq(a) + sq(b) + sq(c));
#undef sq
}

double evalContinuous(float yaw, float pitch, float roll,
					  float gx, float gy, float gz,
					  float ax, float ay, float az,
					  float* gravity) {
	double delta_x, delta_y, delta_z;
	double ypr_eval, gyro_eval, accel_eval, gravity_eval;

//	delta_x = abs(yaw) - THRESHOLD_YAW;
//	delta_y = abs(pitch) - THRESHOLD_PITCH;
//	delta_z = abs(roll) - THRESHOLD_ROLL;
//	ypr_eval = norm(delta_x, delta_y, delta_z);
//
//	delta_x = abs(gyroSmoothend[0]) - THRESHOLD_SMOOTH_GYRO_X;
//	delta_y = abs(gyroSmoothend[1]) - THRESHOLD_SMOOTH_GYRO_Y;
//	delta_z = abs(gyroSmoothend[2]) - THRESHOLD_SMOOTH_GYRO_Z;
//	gyro_eval = norm(delta_x, delta_y, delta_z);
//
//	delta_x = abs(accelIntegral[0]) > THRESHOLD_INT_ACCEL_X;
//	delta_y = abs(accelIntegral[1]) > THRESHOLD_INT_ACCEL_Y;
//	delta_z = abs(accelIntegral[2]) > THRESHOLD_INT_ACCEL_Z;
//	accel_eval = norm(delta_x, delta_y, delta_z);
//
//	delta_x = abs(accelGravityIntegral[0]) > THRESHOLD_INT_GRAVITY_X;
//	delta_y = abs(accelGravityIntegral[1]) > THRESHOLD_INT_GRAVITY_Y;
//	delta_z = abs(accelGravityIntegral[2]) > THRESHOLD_INT_GRAVITY_Z;
//	gravity_eval = norm(delta_x, delta_y, delta_z);
//
//	continuous_rating = ypr_eval + gyro_eval + accel_eval + gravity_eval;
//	normalized_continuous_rating = normalized_continuous_rating * 0.999 + continuous_rating * 0.001;
//
//	return sigmoid_function(continuous_rating/normalized_continuous_rating - 1);

//	double stepCount;
//
//	stepCount = 0 + 1 * (abs(ypr[1]) > THRESHOLD_PITCH * 45 / 100)
//			      + 1 * (abs(ypr[2]) > THRESHOLD_ROLL * 50 / 100)
//				  + 1 * (abs(gyroSmoothend[0]) > THRESHOLD_SMOOTH_GYRO_X * 200 / 100)
//				  + 1 * (abs(gyroSmoothend[1]) > THRESHOLD_SMOOTH_GYRO_Y * 200 / 100)
//				  + 1 * (abs(gyroSmoothend[2]) > THRESHOLD_SMOOTH_GYRO_Z * 200 / 100)
//				  + 1 * (abs(accelIntegral[0]) > THRESHOLD_INT_ACCEL_X * 10000/ 100)
//				  + 1 * (abs(accelIntegral[1]) > THRESHOLD_INT_ACCEL_Y * 10000/ 100)
//				  + 1 * (abs(accelIntegral[2]) > THRESHOLD_INT_ACCEL_Z * 10000/ 100)
//				  + 1 * (abs(accelGravityIntegral[0]) > THRESHOLD_INT_GRAVITY_X * 10000 / 100)
//				  + 1 * (abs(accelGravityIntegral[1]) > THRESHOLD_INT_GRAVITY_Y * 10000 / 100)
//				  + 1 * (abs(acc_vsm) > THRESHOLD_INT_GRAVITY_X * 10000 / 100);
#define ABS_CAST(X) abs((int)(X))
	float stepCount = 0;
	stepCount = 0
			+ 1 * checkThreshold(ABS_CAST(ypr[1] * 180 / M_PI), THRESHOLD_PITCH)
			+ 1 * checkThreshold(ABS_CAST(ypr[2] * 180 / M_PI), THRESHOLD_ROLL)
			+ 1 * checkThreshold(ABS_CAST(gyroSmoothend[0]), THRESHOLD_SMOOTH_GYRO_X)
			+ 1 * checkThreshold(ABS_CAST(gyroSmoothend[1]), THRESHOLD_SMOOTH_GYRO_Y)
			+ 1 * checkThreshold(ABS_CAST(gyroSmoothend[2]), THRESHOLD_SMOOTH_GYRO_Z)
			+ 1 * checkThreshold(ABS_CAST(accelIntegral[0]), THRESHOLD_INT_ACCEL_X)
			+ 1 * checkThreshold(ABS_CAST(accelIntegral[1]), THRESHOLD_INT_ACCEL_Y)
			+ 1 * checkThreshold(ABS_CAST(accelIntegral[2]), THRESHOLD_INT_ACCEL_Z)
			+ 1 * checkThreshold(ABS_CAST(accelGravityIntegral[0]), THRESHOLD_INT_GRAVITY_X)
			+ 1 * checkThreshold(ABS_CAST(accelGravityIntegral[1]), THRESHOLD_INT_GRAVITY_Y)
		 	+ 1 * checkThreshold(ABS_CAST(acc_vsm), THRESHOLD_ACCEL_DELTA);
#undef ABS_CAST
//	for(int i = 0; i < 11; i++){
//		sprintf(str,
//					  "%d ",
//					  chk[i]);
//		HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
//	}

//	sprintf(str,
//				  "%d %2.2f\r\n",
//				  (abs((int)(ypr[1] * 180 / M_PI))),
//				  THRESHOLD_PITCH);
//	HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);

	return stepCount / 11;
}

void prepareData(){
	double delta_x, delta_y, delta_z;

//	rotateZ(rotateZAngle, acceleration_mg, aa_rot);
//	rotateZ(rotateZAngle, aaReal, aaReal_rot);
//	rotateZ(rotateZAngle, angular_rate_mdps, gy_rot);
//	rotateZ(rotateZAngle, ypr, ypr_rot);

	for(register uint8_t i = 0; i < 3; i++)
		accelIntegral[i] = accelIntegral[i] * (1 - MOVING_AVERAGE_DECLINE) + aaReal[i] * MOVING_AVERAGE_DECLINE;

	for(register uint8_t i = 0; i < 3; i++)
		accelGravityIntegral[i] = accelGravityIntegral[i] * (1 - MOVING_AVERAGE_DECLINE) + data_raw_acceleration[i] * MOVING_AVERAGE_DECLINE;

	for(register uint8_t i = 0; i < 3; i++)
		gyroSmoothend[i] = gyroSmoothend[i] * (1 - MOVING_AVERAGE_DECLINE) + data_raw_angular_rate[i] * MOVING_AVERAGE_DECLINE;

	delta_x = former_x - accelIntegral[0];
	delta_y = former_y - accelIntegral[1];
	delta_z = former_z - accelIntegral[2];

	acc_vsm = norm(delta_x, delta_y, delta_z);

	former_x = accelIntegral[0] * MOVING_AVERAGE_DECLINE + former_x * (1 - MOVING_AVERAGE_DECLINE);
	former_y = accelIntegral[1] * MOVING_AVERAGE_DECLINE + former_y * (1 - MOVING_AVERAGE_DECLINE);
	former_z = accelIntegral[2] * MOVING_AVERAGE_DECLINE + former_z * (1 - MOVING_AVERAGE_DECLINE);

}

void rotateZ(float radAngle, float* point, float* rotatedPoint){
	*rotatedPoint = cos(radAngle)*point[0] - sin(radAngle) * point[1];
	*(rotatedPoint+1) = sin(radAngle)*point[0] + cos(radAngle)*point[1];
	*(rotatedPoint+2) = point[2];
}

_Bool checkThreshold(float liveData, float thresholdVal){
	static int i = 0;
	chk[i] = (liveData > thresholdVal);
	if(++i >= 11)
		i = 0;
	return chk[i];
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
