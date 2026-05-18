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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* printf 依赖标准输入输出接口，这里用于串口打印调试信息。 */
#include <stdio.h>
/* AS5600 驱动头文件，提供 I2C 读取角度/状态接口。 */
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
/* 基于 HAL_GetTick() 的非阻塞定时基准：
 * - g_led_tick 控制 LED 心跳闪烁
 * - g_print_tick 控制串口周期打印
 * 这样写比 HAL_Delay() 更利于在主循环里并行处理多任务。 */
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
/* 部分编译环境中，printf 最终会逐字符调用 __io_putchar。
 * 这里把每个字符重定向到 USART2，便于串口终端查看输出。 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1U, 100U);
  return ch;
}

/* GCC/newlib 常通过 _write 完成 printf 输出。
 * 这里把缓冲区整体转发到 USART2。
 * 注：__io_putchar 与 _write 通常保留一种即可，但两者并存可兼容不同工具链配置。 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100U);
  return len;
}

static HAL_StatusTypeDef Read_AdcRaw(uint16_t *adc_raw)
{
  /* 防御式编程：先检查输出指针。 */
  if (adc_raw == NULL)
  {
    return HAL_ERROR;
  }

  /* ADC 单次采样流程第 1 步：启动 ADC。 */
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 第 2 步：等待转换完成。
   * timeout=10ms 表示最多阻塞等待 10ms。 */
  HAL_StatusTypeDef ret = HAL_ADC_PollForConversion(&hadc1, 10U);
  if (ret != HAL_OK)
  {
    /* 等待失败也要 Stop，避免 ADC 保持在不期望状态。 */
    (void)HAL_ADC_Stop(&hadc1);
    return ret;
  }

  /* 第 3 步：读取转换结果（12 位，0~4095）。 */
  *adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

  /* 第 4 步：停止 ADC，完成一次“单次采样”闭环。 */
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static float Angle_ErrorDeg(float adc_angle, float i2c_angle)
{
  /* 先计算原始差值。 */
  float error = adc_angle - i2c_angle;

  /* 角度是环形量，误差要折叠到 [-180, +180]：
   * 例如 359° 与 1° 实际误差应为 -2° 或 +2°，而不是 358°。 */
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

  /* HAL_UART_Receive 是 HAL 的阻塞式接收 API。
   * 这里 timeout=0U，表示“当前时刻立即尝试一次”，没有数据就立刻返回，
   * 不会在此处长时间等待，因此整体效果是主循环里的轮询式接收。 */
  /* 每次只读 1 字节，适合简单单字符命令调试，不适合高速/大量/不定长数据流。
   * 后续学习可扩展到 HAL_UART_Receive_IT 或 HAL_UARTEx_ReceiveToIdle_DMA。 */
  if (HAL_UART_Receive(&huart2, &rx, 1U, 0U) == HAL_OK)
  {
    /* 收到 't'：启动 3 秒开环测试流程。 */
    if (rx == 't')
    {
      MotorTest_Start(now);
    }
    /* 收到 'x'：立即停止测试并回到安全状态。 */
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* 直接发送字符串：用于确认 USART2 外设与引脚配置正常。 */
  HAL_UART_Transmit(&huart2, (uint8_t *)"UART direct test\r\n", 18, 100);
  /* printf 测试：用于确认重定向链路（__io_putchar/_write）正常。 */
  printf("printf test\r\n");

  MotorDriver_Init();

  /* STM32F1 的 ADC 在使用前建议校准，可减小转换偏差。 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    printf("[ERR] ADC calibration failed\r\n");
  }
  else
  {
    printf("[OK ] ADC calibration done\r\n");
  }

  printf("AS5600 ADC/I2C compare start\r\n");
  /* 从当前系统 tick 开始计时，避免初次比较时出现异常大时间差。 */
  g_led_tick = HAL_GetTick();
  g_print_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* HAL_GetTick() 返回系统运行毫秒数，是主循环非阻塞调度的基础。 */
    uint32_t now = HAL_GetTick();

    /* 每轮主循环都检查一次串口命令，无需用 HAL_Delay 阻塞等待串口数据。 */
    MotorTest_HandleUartCommand(now);
    MotorTest_Update(now);

    /* 每 500ms 翻转 LED，作为“程序仍在运行”的心跳指示。 */
    if ((now - g_led_tick) >= 500U)
    {
      HAL_GPIO_TogglePin(PC13_RUN_LED_GPIO_Port, PC13_RUN_LED_Pin);
      g_led_tick = now;
    }

    /* 每 200ms 打印一次 ADC 与 I2C 角度对比。 */
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
      /* 读取流程：
       * 1) ADC 读模拟角度电压对应的原始值
       * 2) I2C 读 AS5600 的 RAW_ANGLE */
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
        /* STM32F103 ADC 为 12 位：0~4095 -> 0~360 度。 */
        adc_angle = ((float)adc_raw * 360.0f) / 4095.0f;
        /* AS5600 也是 12 位，使用驱动提供的统一换算函数。 */
        i2c_angle = AS5600_RawToDegree(i2c_raw);
        /* 计算环形角度误差（限制在 -180~+180 度）。 */
        error = Angle_ErrorDeg(adc_angle, i2c_angle);

        /* 用“角度×100”的整数打印，规避部分嵌入式环境 float printf 额外链接配置问题。
         * 例如 12345 表示 123.45 度。 */
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
