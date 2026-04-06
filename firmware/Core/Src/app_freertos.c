/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "foc_cfg.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StartHandle;
uint32_t StartBuffer[ 1280 ];
osStaticThreadDef_t StartControlBlock;
osThreadId oledReflashHandle;
uint32_t oledReflashBuffer[ 256 ];
osStaticThreadDef_t oledReflashControlBlock;
osThreadId KeyScanHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId TempSenseHandle;
/* USER CODE END FunctionPrototypes */

void StartTask(void const * argument);
extern void oledReflashTask(void const * argument);
void KeyScanTask(void const * argument);
extern void TempSenseTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  /* definition and creation of Start */
  osThreadStaticDef(Start, StartTask, osPriorityNormal, 0, 1280, StartBuffer, &StartControlBlock);
  StartHandle = osThreadCreate(osThread(Start), NULL);

  /* definition and creation of oledReflash */
  osThreadStaticDef(oledReflash, oledReflashTask, osPriorityLow, 0, 256, oledReflashBuffer, &oledReflashControlBlock);
  oledReflashHandle = osThreadCreate(osThread(oledReflash), NULL);

  /* definition and creation of KeyScan */
  osThreadDef(KeyScan, KeyScanTask, osPriorityIdle, 0, 128);
  KeyScanHandle = osThreadCreate(osThread(KeyScan), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(TempSense, TempSenseTask, osPriorityIdle, 0, 128);
  TempSenseHandle = osThreadCreate(osThread(TempSense), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask */
/**
  * @brief  Function implementing the Start thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask */
#include "vofa.h"
__weak void StartTask(void const * argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN StartTask */
//  vTaskDelete(StartHandle);
  /* Infinite loop */
  for(;;)
  {
//      VofaStart();
      osDelay(1);
  }
  /* USER CODE END StartTask */
}

/* USER CODE BEGIN Header_KeyScanTask */
/**
  * @brief  Function implementing the KeyScan thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_KeyScanTask */
__weak void KeyScanTask(void const * argument)
{
  /* USER CODE BEGIN KeyScanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyScanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

