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
osThreadId Init_TaskHandleHandle;
osThreadId Comm_TaskHandleHandle;
osThreadId Ins_TaskHandleHandle;
osThreadId WatchDog_TaskHandleHandle;
osThreadId Shoot_TaskHandleHandle;
osThreadId Remote_TaskHandleHandle;
osThreadId Gimbal_TaskHandleHandle;
osThreadId Referee_TaskHandleHandle;
osThreadId Chassis_TaskHandleHandle;
osThreadId Power_LimitingHandle;
osTimerId SoftTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const * argument);
void Comm_Task(void const * argument);
void Ins_Task(void const * argument);
void WatchDog_Task(void const * argument);
void Shoot_Task(void const * argument);
void Remote_Task(void const * argument);
void Gimbal_Task(void const * argument);
void Referee_Task(void const * argument);
void Chassis_Task(void const * argument);
void SoftTimerCallback(void const * argument);
void Power_Limting_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of SoftTimer */
  osTimerDef(SoftTimer, SoftTimerCallback);
  SoftTimerHandle = osTimerCreate(osTimer(SoftTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Init_TaskHandle */
  osThreadDef(Init_TaskHandler, Init_Task, osPriorityRealtime, 0, 128);
  Init_TaskHandleHandle = osThreadCreate(osThread(Init_TaskHandler), NULL);

  /* definition and creation of Comm_TaskHandle */
  osThreadDef(Comm_TaskHandler, Comm_Task, osPriorityRealtime, 0, 128);
  Comm_TaskHandleHandle = osThreadCreate(osThread(Comm_TaskHandler), NULL);

  /* definition and creation of Ins_TaskHandler */
  osThreadDef(Ins_TaskHandler, Ins_Task, osPriorityRealtime, 0, 128);
  Ins_TaskHandleHandle = osThreadCreate(osThread(Ins_TaskHandler), NULL);

  /* definition and creation of WatchDog_TaskHa */
  osThreadDef(WatchDog_TaskHandler, WatchDog_Task, osPriorityHigh, 0, 128);
  WatchDog_TaskHandleHandle = osThreadCreate(osThread(WatchDog_TaskHandler), NULL);
  
  /* definition and creation of Shoot_TaskHandl */
  osThreadDef(Shoot_TaskHandler, Shoot_Task, osPriorityRealtime, 0, 128);
  Shoot_TaskHandleHandle = osThreadCreate(osThread(Shoot_TaskHandler), NULL);

  /* definition and creation of Remote_TaskHand */
  osThreadDef(Remote_TaskHandler, Remote_Task, osPriorityRealtime, 0, 128);
  Remote_TaskHandleHandle = osThreadCreate(osThread(Remote_TaskHandler), NULL);

  /* definition and creation of Gimbal_TaskHand */
  osThreadDef(Gimbal_TaskHandler, Gimbal_Task, osPriorityRealtime, 0, 128);
  Gimbal_TaskHandleHandle = osThreadCreate(osThread(Gimbal_TaskHandler), NULL);

  /* definition and creation of Referee_TaskHa */
  osThreadDef(Referee_TaskHandler, Referee_Task, osPriorityRealtime, 0, 128);
  Referee_TaskHandleHandle = osThreadCreate(osThread(Referee_TaskHandler), NULL);
	
  /* definition and creation of Referee_TaskHa */
  osThreadDef(Chassis_TaskHandler, Chassis_Task, osPriorityRealtime, 0, 128);
  Chassis_TaskHandleHandle = osThreadCreate(osThread(Chassis_TaskHandler), NULL);
	
  osThreadDef(Power_limitHandle, Power_Limting_Task, osPriorityRealtime, 0, 128);
  Power_LimitingHandle = osThreadCreate(osThread(Power_limitHandle), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
  * @brief  Function implementing the Init_TaskHandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Comm_Task */
/**
* @brief Function implementing the Comm_TaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Comm_Task */
__weak void Comm_Task(void const * argument)
{
  /* USER CODE BEGIN Comm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Comm_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins_TaskHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
__weak void WatchDog_Task(void const * argument)
{
  /* USER CODE BEGIN WatchDog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WatchDog_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot_TaskHandl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
__weak void Referee_Task(void const * argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* SoftTimerCallback function */
__weak void SoftTimerCallback(void const * argument)
{
  /* USER CODE BEGIN SoftTimerCallback */

  /* USER CODE END SoftTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
