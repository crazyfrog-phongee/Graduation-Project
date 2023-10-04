/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "adc.h"
#include "sx1278.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VREFINT (1.21)
#define ADC_RESOLUTION (4095.0)
#define RATIO (1.3)
#define VCC (3.3)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const char *TAG = "FREERTOS";
extern sx1278_node_t sx1278_node;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void peripheral_task(void const *arg);
void sx1278_task(void const *arg);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate((TaskFunction_t)(&peripheral_task), "PERIPHERAL", 256, NULL, 10, NULL);
  xTaskCreate((TaskFunction_t)(&sx1278_task), "SX1278", 256, NULL, 9, NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  char data_log[20];
  sprintf(data_log, "Default task");

  /* Infinite loop */
  for (;;)
  {
    LOG(TAG, data_log);
    HAL_Delay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void peripheral_task(void const *arg)
{
  /* USER CODE BEGIN StartDefaultTask */
  static uint32_t adc_val[22] = {0};
  static uint16_t ADC_VREF_mV = 3300;
  static float battery = 0;
  char data_log[40];

  memset(data_log, 0, 40 * sizeof(char));
  sprintf(data_log, "Peripheral task");
  LOG(TAG, data_log);

  volatile uint32_t time_keeper_1 = HAL_GetTick();
  /* Infinite loop */
  for (;;)
  {
    HAL_ADC_Start_DMA(&hadc1, adc_val, 20);
    time_keeper_1 = HAL_GetTick();
    while ((HAL_GetTick() - time_keeper_1) <= 3000)
    {
      adc_val[20] = 0;
      adc_val[21] = 0;
      for (uint8_t i = 0; i < 20; i += 2)
      {
        adc_val[20] += adc_val[i];
        adc_val[21] += adc_val[i + 1];
      }
      adc_val[20] /= 10;
      adc_val[21] /= 10;

      ADC_VREF_mV = (uint16_t)(VREFINT * ADC_RESOLUTION * 1000 / adc_val[21]) - 130;
      battery = (float)(((float)adc_val[20] * RATIO * ADC_VREF_mV / ADC_RESOLUTION) / 1000);
      ftoa(battery, sx1278_node.battery, 2);

      /* Logging data */
      sprintf(data_log, "Vref: %d, Vbat: %s", ADC_VREF_mV, sx1278_node.battery);
      LOG(TAG, data_log);
      HAL_Delay(1000);
    }

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_Delay(2000);
  }
  /* USER CODE END StartDefaultTask */
}

void sx1278_task(void const *arg)
{
  /* USER CODE BEGIN StartDefaultTask */
  char data_log[40] = {0};

  sprintf(data_log, "SX1278 Task");
  LOG(TAG, data_log);

  sprintf(data_log, "for loop in sx1278_task");
  for (;;)
  {
    LOG(TAG, data_log);
    HAL_Delay(500);
  }

  /* USER CODE END StartDefaultTask */
}
/* USER CODE END Application */
