/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "as5600.h"
#include "motor_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_TEST_DURATION_MS 3000U
#define MOTOR_TEST_STEP_MS     20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t g_led_tick = 0U;
static uint32_t g_print_tick = 0U;
static uint8_t g_motor_test_active = 0U;
static uint8_t g_motor_test_step = 0U;
static uint32_t g_motor_test_start_tick = 0U;
static uint32_t g_motor_test_step_tick = 0U;

static const float kMotorTestDutyTable[6][3] = {
  {0.60f, 0.40f, 0.50f},
  {0.60f, 0.50f, 0.40f},
  {0.50f, 0.60f, 0.40f},
  {0.40f, 0.60f, 0.50f},
  {0.40f, 0.50f, 0.60f},
  {0.50f, 0.40f, 0.60f},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef Read_AdcRaw(uint16_t *adc_raw);
static float Angle_ErrorDeg(float adc_angle, float i2c_angle);
static void MotorTest_Start(uint32_t now);
static void MotorTest_Stop(void);
static void MotorTest_Update(uint32_t now);
static void MotorTest_HandleUartCommand(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1U, 100U);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100U);
  return len;
}

static HAL_StatusTypeDef Read_AdcRaw(uint16_t *adc_raw)
{
  if (adc_raw == NULL)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef ret = HAL_ADC_PollForConversion(&hadc1, 10U);
  if (ret != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return ret;
  }

  *adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static float Angle_ErrorDeg(float adc_angle, float i2c_angle)
{
  float error = adc_angle - i2c_angle;

  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }

  return error;
}

static void MotorTest_Start(uint32_t now)
{
  MotorDriver_SetAllPwmZero();
  g_motor_test_active = 1U;
  g_motor_test_start_tick = now;
  g_motor_test_step_tick = now;
  g_motor_test_step = 0U;
  MotorDriver_Enable();
  printf("[MOTOR] open-loop test start\r\n");
}

static void MotorTest_Stop(void)
{
  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_motor_test_active = 0U;
  printf("[MOTOR] open-loop test stop\r\n");
}

static void MotorTest_Update(uint32_t now)
{
  if (g_motor_test_active == 0U)
  {
    return;
  }

  if ((now - g_motor_test_start_tick) >= MOTOR_TEST_DURATION_MS)
  {
    MotorTest_Stop();
    return;
  }

  if ((now - g_motor_test_step_tick) >= MOTOR_TEST_STEP_MS)
  {
    MotorDriver_SetPwmDuty(kMotorTestDutyTable[g_motor_test_step][0],
                           kMotorTestDutyTable[g_motor_test_step][1],
                           kMotorTestDutyTable[g_motor_test_step][2]);
    g_motor_test_step = (uint8_t)((g_motor_test_step + 1U) % 6U);
    g_motor_test_step_tick = now;
  }
}

static void MotorTest_HandleUartCommand(uint32_t now)
{
  uint8_t rx = 0U;

  if (HAL_UART_Receive(&huart2, &rx, 1U, 0U) == HAL_OK)
  {
    if (rx == 't')
    {
      MotorTest_Start(now);
    }
    else if (rx == 'x')
    {
      MotorTest_Stop();
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, (uint8_t *)"UART direct test\r\n", 18, 100);
  printf("printf test\r\n");

  MotorDriver_Init();

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    printf("[ERR] ADC calibration failed\r\n");
  }
  else
  {
    printf("[OK ] ADC calibration done\r\n");
  }

  printf("AS5600 ADC/I2C compare start\r\n");
  g_led_tick = HAL_GetTick();
  g_print_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    MotorTest_HandleUartCommand(now);
    MotorTest_Update(now);

    if ((now - g_led_tick) >= 500U)
    {
      HAL_GPIO_TogglePin(PC13_RUN_LED_GPIO_Port, PC13_RUN_LED_Pin);
      g_led_tick = now;
    }

    if ((now - g_print_tick) >= 200U)
    {
      uint16_t adc_raw = 0U;
      uint16_t i2c_raw = 0U;
      float adc_angle = 0.0f;
      float i2c_angle = 0.0f;
      float error = 0.0f;
      int32_t adc_angle_x100 = 0;
      int32_t i2c_angle_x100 = 0;
      int32_t error_x100 = 0;
      HAL_StatusTypeDef adc_ret = Read_AdcRaw(&adc_raw);
      HAL_StatusTypeDef i2c_ret = AS5600_ReadRawAngle(&hi2c1, &i2c_raw);

      if (adc_ret != HAL_OK)
      {
        printf("[ERR] ADC read failed, ret=%d\r\n", adc_ret);
      }

      if (i2c_ret != HAL_OK)
      {
        printf("[ERR] AS5600 I2C read failed, ret=%d, i2c_err=0x%08lX\r\n", i2c_ret, hi2c1.ErrorCode);
      }

      if ((adc_ret == HAL_OK) && (i2c_ret == HAL_OK))
      {
        adc_angle = ((float)adc_raw * 360.0f) / 4095.0f;
        i2c_angle = AS5600_RawToDegree(i2c_raw);
        error = Angle_ErrorDeg(adc_angle, i2c_angle);

        adc_angle_x100 = (int32_t)(adc_angle * 100.0f);
        i2c_angle_x100 = (int32_t)(i2c_angle * 100.0f);
        error_x100 = (int32_t)(error * 100.0f);

        printf("ADC_raw=%u, I2C_raw=%u, ADC_angle_x100=%ld, I2C_angle_x100=%ld, error_x100=%ld\r\n",
               adc_raw, i2c_raw, adc_angle_x100, i2c_angle_x100, error_x100);
      }

      g_print_tick = now;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
