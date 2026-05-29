/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "motor_control_config.h"
#include "motor_feedback.h"
#include "motor_ui.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_RTOS_STACK_MONITOR 0U
/*
 * Stage-2 RTOS ownership:
 * - FeedbackTask owns I2C1/AS5600 sampling.
 * - UITask owns LCD/touch UI work and reads feedback snapshots.
 * - ControlTask consumes snapshots through motor_control and never touches I2C/SPI.
 * Mutexes are deferred until a peripheral is shared by more than one task.
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FeedbackTask */
osThreadId_t FeedbackTaskHandle;
const osThreadAttr_t FeedbackTask_attributes = {
  .name = "FeedbackTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if ENABLE_RTOS_STACK_MONITOR
static void RtosStackMonitor_Sample(void);
#endif

/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartUiTask(void *argument);
void StartMonitorTask(void *argument);
void StartFeedbackTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of UITask */
  UITaskHandle = osThreadNew(StartUiTask, NULL, &UITask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of FeedbackTask */
  FeedbackTaskHandle = osThreadNew(StartFeedbackTask, NULL, &FeedbackTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if ((ControlTaskHandle == NULL) ||
      (FeedbackTaskHandle == NULL) ||
      (UITaskHandle == NULL) ||
      (MonitorTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  (void)argument;
  uint32_t wake_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    wake_tick += 1U;
    MotorControl_Tick1ms();
    (void)osDelayUntil(wake_tick);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUiTask */
void StartUiTask(void *argument)
{
  /* USER CODE BEGIN StartUiTask */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    MotorUi_Update(HAL_GetTick());
    osDelay(10);
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(PC13_RUN_LED_GPIO_Port, PC13_RUN_LED_Pin);
#if ENABLE_RTOS_STACK_MONITOR
    RtosStackMonitor_Sample();
#endif
    osDelay(500);
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartFeedbackTask */
/**
* @brief Function implementing the FeedbackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFeedbackTask */
void StartFeedbackTask(void *argument)
{
  /* USER CODE BEGIN StartFeedbackTask */
  (void)argument;
  uint32_t wake_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    wake_tick += MOTOR_FEEDBACK_TASK_PERIOD_MS;
    MotorFeedback_Update(HAL_GetTick());
    (void)osDelayUntil(wake_tick);
  }
  /* USER CODE END StartFeedbackTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#if ENABLE_RTOS_STACK_MONITOR
static void RtosStackMonitor_Sample(void)
{
  volatile UBaseType_t control_stack_words = uxTaskGetStackHighWaterMark(ControlTaskHandle);
  volatile UBaseType_t feedback_stack_words = uxTaskGetStackHighWaterMark(FeedbackTaskHandle);
  volatile UBaseType_t ui_stack_words = uxTaskGetStackHighWaterMark(UITaskHandle);
  volatile UBaseType_t monitor_stack_words = uxTaskGetStackHighWaterMark(MonitorTaskHandle);

  (void)control_stack_words;
  (void)feedback_stack_words;
  (void)ui_stack_words;
  (void)monitor_stack_words;
}
#endif

/* USER CODE END Application */
