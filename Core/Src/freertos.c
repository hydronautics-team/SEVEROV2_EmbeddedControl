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
#include "semphr.h"
#include "usart.h"
#include "i2c.h"
#include "timers.h"
#include "messages.h"
#include "robot.h"
#include "global.h"
#include "communication.h"
#include "stabilization.h"
#include "checksum.h"
#include "thrusters.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
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
#define SHORE_DELAY	  45

TimerHandle_t UARTTimer;
TimerHandle_t SilenceTimer;

uint8_t silence_ticks = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tLedBlinkingTask */
osThreadId_t tLedBlinkingTaskHandle;
uint32_t tLedBlinkingTaskBuffer[ 128 ];
osStaticThreadDef_t tLedBlinkingTaskControlBlock;
const osThreadAttr_t tLedBlinkingTask_attributes = {
  .name = "tLedBlinkingTask",
  .cb_mem = &tLedBlinkingTaskControlBlock,
  .cb_size = sizeof(tLedBlinkingTaskControlBlock),
  .stack_mem = &tLedBlinkingTaskBuffer[0],
  .stack_size = sizeof(tLedBlinkingTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tVmaCommTask */
osThreadId_t tVmaCommTaskHandle;
uint32_t tVmaCommTaskBuffer[ 128 ];
osStaticThreadDef_t tVmaCommTaskControlBlock;
const osThreadAttr_t tVmaCommTask_attributes = {
  .name = "tVmaCommTask",
  .cb_mem = &tVmaCommTaskControlBlock,
  .cb_size = sizeof(tVmaCommTaskControlBlock),
  .stack_mem = &tVmaCommTaskBuffer[0],
  .stack_size = sizeof(tVmaCommTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tImuCommTask */
osThreadId_t tImuCommTaskHandle;
uint32_t tImuCommTaskBuffer[ 128 ];
osStaticThreadDef_t tImuCommTaskControlBlock;
const osThreadAttr_t tImuCommTask_attributes = {
  .name = "tImuCommTask",
  .cb_mem = &tImuCommTaskControlBlock,
  .cb_size = sizeof(tImuCommTaskControlBlock),
  .stack_mem = &tImuCommTaskBuffer[0],
  .stack_size = sizeof(tImuCommTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tStabilizationTask */
osThreadId_t tStabilizationTaskHandle;
uint32_t tStabilizationTaskBuffer[ 128 ];
osStaticThreadDef_t tStabilizationTaskControlBlock;
const osThreadAttr_t tStabilizationTask_attributes = {
  .name = "tStabilizationTask",
  .cb_mem = &tStabilizationTaskControlBlock,
  .cb_size = sizeof(tStabilizationTaskControlBlock),
  .stack_mem = &tStabilizationTaskBuffer[0],
  .stack_size = sizeof(tStabilizationTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tDevCommTask */
osThreadId_t tDevCommTaskHandle;
uint32_t tDevCommTaskBuffer[ 128 ];
osStaticThreadDef_t tDevCommTaskControlBlock;
const osThreadAttr_t tDevCommTask_attributes = {
  .name = "tDevCommTask",
  .cb_mem = &tDevCommTaskControlBlock,
  .cb_size = sizeof(tDevCommTaskControlBlock),
  .stack_mem = &tDevCommTaskBuffer[0],
  .stack_size = sizeof(tDevCommTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tSensCommTask */
osThreadId_t tSensCommTaskHandle;
uint32_t tSensCommTaskBuffer[ 128 ];
osStaticThreadDef_t tSensCommTaskControlBlock;
const osThreadAttr_t tSensCommTask_attributes = {
  .name = "tSensCommTask",
  .cb_mem = &tSensCommTaskControlBlock,
  .cb_size = sizeof(tSensCommTaskControlBlock),
  .stack_mem = &tSensCommTaskBuffer[0],
  .stack_size = sizeof(tSensCommTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tPcCommTask */
osThreadId_t tPcCommTaskHandle;
uint32_t tPcCommTaskBuffer[ 128 ];
osStaticThreadDef_t tPcCommTaskControlBlock;
const osThreadAttr_t tPcCommTask_attributes = {
  .name = "tPcCommTask",
  .cb_mem = &tPcCommTaskControlBlock,
  .cb_size = sizeof(tPcCommTaskControlBlock),
  .stack_mem = &tPcCommTaskBuffer[0],
  .stack_size = sizeof(tPcCommTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tUartTimer */
osTimerId_t tUartTimerHandle;
const osTimerAttr_t tUartTimer_attributes = {
  .name = "tUartTimer"
};
/* Definitions for tSilence */
osTimerId_t tSilenceHandle;
const osTimerAttr_t tSilence_attributes = {
  .name = "tSilence"
};
/* Definitions for tTechCommTImer */
osTimerId_t tTechCommTImerHandle;
const osTimerAttr_t tTechCommTImer_attributes = {
  .name = "tTechCommTImer"
};
/* Definitions for mutData */
osMutexId_t mutDataHandle;
osStaticMutexDef_t mutDataControlBlock;
const osMutexAttr_t mutData_attributes = {
  .name = "mutData",
  .cb_mem = &mutDataControlBlock,
  .cb_size = sizeof(mutDataControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void nullIntArray(uint8_t *array, uint8_t size);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void func_tLedBlinkingTask(void *argument);
void func_tVmaCommTask(void *argument);
void func_tImuCommTask(void *argument);
void func_tStabilizationTask(void *argument);
void func_tDevCommTask(void *argument);
void func_tSensCommTask(void *argument);
void func_tPcCommTask(void *argument);
void func_tUartTimer(void *argument);
void tSilence_func(void *argument);
void tTechCommTImer_callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutData */
  mutDataHandle = osMutexNew(&mutData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of tUartTimer */
  tUartTimerHandle = osTimerNew(func_tUartTimer, osTimerOnce, NULL, &tUartTimer_attributes);

  /* creation of tSilence */
  tSilenceHandle = osTimerNew(tSilence_func, osTimerOnce, NULL, &tSilence_attributes);

  /* creation of tTechCommTImer */
  tTechCommTImerHandle = osTimerNew(tTechCommTImer_callback, osTimerOnce, NULL, &tTechCommTImer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_TIMERS */
  SilenceTimer = xTimerCreate("silence", DELAY_SILENCE/portTICK_RATE_MS, pdFALSE, 0, (TimerCallbackFunction_t) tSilence_func);
  UARTTimer = xTimerCreate("timer", DELAY_TIMER_TASK/portTICK_RATE_MS, pdFALSE, 0, (TimerCallbackFunction_t) func_tUartTimer);

  xTimerStart(SilenceTimer, 10);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of tLedBlinkingTask */
  tLedBlinkingTaskHandle = osThreadNew(func_tLedBlinkingTask, NULL, &tLedBlinkingTask_attributes);

  /* creation of tVmaCommTask */
  tVmaCommTaskHandle = osThreadNew(func_tVmaCommTask, NULL, &tVmaCommTask_attributes);

  /* creation of tImuCommTask */
  tImuCommTaskHandle = osThreadNew(func_tImuCommTask, NULL, &tImuCommTask_attributes);

  /* creation of tStabilizationTask */
  tStabilizationTaskHandle = osThreadNew(func_tStabilizationTask, NULL, &tStabilizationTask_attributes);

  /* creation of tDevCommTask */
  tDevCommTaskHandle = osThreadNew(func_tDevCommTask, NULL, &tDevCommTask_attributes);

  /* creation of tSensCommTask */
  tSensCommTaskHandle = osThreadNew(func_tSensCommTask, NULL, &tSensCommTask_attributes);

  /* creation of tPcCommTask */
  tPcCommTaskHandle = osThreadNew(func_tPcCommTask, NULL, &tPcCommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_func_tLedBlinkingTask */
/**
* @brief Function implementing the tLedBlinkingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tLedBlinkingTask */
void func_tLedBlinkingTask(void *argument)
{
  /* USER CODE BEGIN func_tLedBlinkingTask */
  /* Infinite loop */
  for(;;)
  {
        HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
        osDelayUntil(DELAY_LED_TASK);
        HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
        osDelayUntil(DELAY_LED_TASK);
        HAL_GPIO_TogglePin(led1_GPIO_Port, led3_Pin);
        osDelayUntil(DELAY_LED_TASK);
  }
  /* USER CODE END func_tLedBlinkingTask */
}

/* USER CODE BEGIN Header_func_tVmaCommTask */
/**
* @brief Function implementing the tVmaCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tVmaCommTask */
void func_tVmaCommTask(void *argument)
{
  /* USER CODE BEGIN func_tVmaCommTask */
	uint8_t transaction = 0;
	/* Infinite loop */
	for(;;)
	{
		if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_THRUSTERS_TASK) == pdTRUE) {
			fillThrustersRequest(ThrustersRequestBuffer, transaction);
			xSemaphoreGive(mutDataHandle);
		}

		uartBus[THRUSTERS_UART].txBuffer = ThrustersRequestBuffer;
		uartBus[THRUSTERS_UART].txLength = THRUSTERS_REQUEST_LENGTH;

		uartBus[THRUSTERS_UART].rxBuffer = ThrustersResponseBuffer[transaction];
		uartBus[THRUSTERS_UART].rxLength = THRUSTERS_RESPONSE_LENGTH;

		transmitAndReceive(&uartBus[THRUSTERS_UART], false);

		if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_THRUSTERS_TASK) == pdTRUE) {
			fillThrustersResponse(ThrustersResponseBuffer[transaction], transaction);
			xSemaphoreGive(mutDataHandle);
		}

		transaction = (transaction + 1) % THRUSTERS_NUMBER;
		osDelayUntil(DELAY_THRUSTERS_TASK);
	}
  /* USER CODE END func_tVmaCommTask */
}

/* USER CODE BEGIN Header_func_tImuCommTask */
/**
* @brief Function implementing the tImuCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tImuCommTask */
void func_tImuCommTask(void *argument)
{
  /* USER CODE BEGIN func_tImuCommTask */
  /* Infinite loop */
  for(;;)
  {
	  	if(rSensors.resetIMU) {
			uartBus[IMU_UART].txBuffer = ImuResetRequestBuffer;
			uartBus[IMU_UART].txLength = IMU_REQUEST_LENGTH;
	  		transmitPackage(&uartBus[IMU_UART], false);

	  		rSensors.pressure_null = rSensors.pressure;
	  		rSensors.resetIMU = false;
	  	}
	  	else {
	  		uartBus[IMU_UART].txBuffer = ImuRequestBuffer;
	  		uartBus[IMU_UART].txLength = IMU_REQUEST_LENGTH;

	  		uartBus[IMU_UART].rxBuffer = ImuResponseBuffer;
	  		uartBus[IMU_UART].rxLength = IMU_RESPONSE_LENGTH*IMU_CHECKSUMS;

	  		HAL_UART_Receive_IT(uartBus[IMU_UART].huart, uartBus[IMU_UART].rxBuffer, uartBus[IMU_UART].rxLength);
	  		HAL_UART_Transmit_IT(uartBus[IMU_UART].huart, uartBus[IMU_UART].txBuffer, uartBus[IMU_UART].txLength);
	  		osDelayUntil(DELAY_IMU_TASK);

	  		//if(transmitAndReceive(&uartBus[IMU_UART], false)) {
	  			if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_IMU_TASK) == pdTRUE) {
	  				ImuReceive(ImuResponseBuffer);
	  				xSemaphoreGive(mutDataHandle);
	  			}
	  		//}

	  	}

	  	osDelayUntil(DELAY_IMU_TASK);
  }
  /* USER CODE END func_tImuCommTask */
}

/* USER CODE BEGIN Header_func_tStabilizationTask */
/**
* @brief Function implementing the tStabilizationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tStabilizationTask */
void func_tStabilizationTask(void *argument)
{
  /* USER CODE BEGIN func_tStabilizationTask */
	/* Infinite loop */
	for(;;)
	{
		if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_STABILIZATION_TASK) == pdTRUE) {
			for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
				if (rStabConstants[i].enable) {
					stabilizationUpdate(i);
				}
			}
			//formThrustVectors();
			xSemaphoreGive(mutDataHandle);
		}

		osDelayUntil(DELAY_STABILIZATION_TASK);
	}
  /* USER CODE END func_tStabilizationTask */
}

/* USER CODE BEGIN Header_func_tDevCommTask */
/**
* @brief Function implementing the tDevCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tDevCommTask */
void func_tDevCommTask(void *argument)
{
  /* USER CODE BEGIN func_tDevCommTask */
    uint8_t transaction = 0;
  /* Infinite loop */
  for(;;)
  {
        if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_DEVICES_TASK) == pdTRUE) {
            DevicesRequestUpdate(DevicesRequestBuffer, transaction);
            xSemaphoreGive(mutDataHandle);
        }

		uartBus[DEVICES_UART].txBuffer = DevicesRequestBuffer;
		uartBus[DEVICES_UART].txLength = DEVICES_REQUEST_LENGTH;

		uartBus[DEVICES_UART].rxBuffer = DevicesResponseBuffer[transaction];
		uartBus[DEVICES_UART].rxLength = DEVICES_RESPONSE_LENGTH;

		transmitAndReceive(&uartBus[DEVICES_UART], false);

        if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_DEVICES_TASK) == pdTRUE) {
            DevicesResponseUpdate(DevicesResponseBuffer[transaction], transaction);
            xSemaphoreGive(mutDataHandle);
        }

        transaction = (transaction + 1) % DEVICES_NUMBER;
        osDelayUntil(DELAY_DEVICES_TASK);
  }
  /* USER CODE END func_tDevCommTask */
}

/* USER CODE BEGIN Header_func_tSensCommTask */
/**
* @brief Function implementing the tSensCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tSensCommTask */
void func_tSensCommTask(void *argument)
{
  /* USER CODE BEGIN func_tSensCommTask */
  /* Infinite loop */
  for(;;)
  {
	  receiveI2cPackageDMA(DEV_I2C, SENSORS_PRESSURE_ADDR, PressureResponseBuffer, PRESSURE_SENSOR_SIZE);
	  osDelayUntil(DELAY_SENSOR_TASK);
	  if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_SENSOR_TASK) == pdTRUE) {
	  	  SensorsResponseUpdate(PressureResponseBuffer, DEV_I2C);
	  	  xSemaphoreGive(mutDataHandle);
	  }
	  osDelayUntil(DELAY_SENSOR_TASK);
  }
  /* USER CODE END func_tSensCommTask */
}

/* USER CODE BEGIN Header_func_tPcCommTask */
/**
* @brief Function implementing the tPcCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_tPcCommTask */
void func_tPcCommTask(void *argument)
{
  /* USER CODE BEGIN func_tPcCommTask */
  /* Infinite loop */
  for(;;)
  {
	  osDelayUntil(DELAY_PC_TASK);
  }
  /* USER CODE END func_tPcCommTask */
}

/* func_tUartTimer function */
void func_tUartTimer(void *argument)
{
  /* USER CODE BEGIN func_tUartTimer */
	if (uartBus[SHORE_UART].packageReceived) {
		bool package = true;
		if(xSemaphoreTake(mutDataHandle, (TickType_t) WAITING_TIMER) == pdTRUE) {
		//	xSemaphoreTake( xSemaphore, xBlockTime )		xQueueSemaphoreTake( ( xSemaphore ), ( xBlockTime ) )
			switch(uartBus[SHORE_UART].rxBuffer[0]) {
				case SHORE_REQUEST_CODE:
					ShoreRequest(uartBus[SHORE_UART].rxBuffer);
					ShoreResponse(uartBus[SHORE_UART].txBuffer);
					uartBus[SHORE_UART].txLength = SHORE_RESPONSE_LENGTH;
					break;
				case REQUEST_CONFIG_CODE:
					ShoreConfigRequest(uartBus[SHORE_UART].rxBuffer);
					ShoreConfigResponse(uartBus[SHORE_UART].txBuffer);
					uartBus[SHORE_UART].txLength = SHORE_CONFIG_RESPONSE_LENGTH;
					break;
				case DIRECT_REQUEST_CODE:
					ShoreDirectRequest(uartBus[SHORE_UART].rxBuffer);
					ShoreDirectResponse(uartBus[SHORE_UART].txBuffer);
					uartBus[SHORE_UART].txLength = SHORE_DIRECT_RESPONSE_LENGTH;
					break;
				default:
					package = false;
			}
			xSemaphoreGive(mutDataHandle);
		}
		if(package) {
			HAL_UART_Transmit_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].txBuffer, uartBus[SHORE_UART].txLength);
		}
	}
	else {
		++uartBus[SHORE_UART].outdatedRxCounter;
	}
	counterRx = 0;
	uartBus[SHORE_UART].packageReceived = false;
	HAL_UART_AbortReceive_IT(uartBus[SHORE_UART].huart);
	HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer, 1);
  /* USER CODE END func_tUartTimer */
}

/* tSilence_func function */
void tSilence_func(void *argument)
{
  /* USER CODE BEGIN tSilence_func */
	if(fromTickToMs(xTaskGetTickCount()) - uartBus[SHORE_UART].lastMessage > UART_SWITCH_DELAY && counterRx == 0) {
//		if(uartBus[SHORE_UART].huart == &huart1) {
//			uartBus[SHORE_UART].huart = &huart5;
//		}
//		else if(uartBus[SHORE_UART].huart == &huart5) {
//			uartBus[SHORE_UART].huart = &huart1;
//		}
		HAL_UART_AbortReceive_IT(uartBus[SHORE_UART].huart);
		HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer, 1);

		if(xSemaphoreTake(mutDataHandle, (TickType_t) WAITING_TIMER) == pdTRUE) {
			resetThrusters();
			for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
				rStabConstants[i].enable = false;
			}
			xSemaphoreGive(mutDataHandle);
		}

//		if(uartBus[SHORE_UART].lastMessage == 0) {
//			rState.pcCounter++;
//			switch(rState.pcCounter) {
//			case PC_POWERON_DELAY:
//				HAL_GPIO_WritePin(PC_CONTROL1_GPIO_Port, PC_CONTROL1_Pin, GPIO_PIN_SET); // RESET
//				HAL_GPIO_WritePin(PC_CONTROL2_GPIO_Port, PC_CONTROL2_Pin, GPIO_PIN_SET); // ONOFF
//				break;
//			case PC_POWERON_DELAY+1:
//			HAL_GPIO_WritePin(PC_CONTROL1_GPIO_Port, PC_CONTROL1_Pin, GPIO_PIN_RESET); // RESET
//			HAL_GPIO_WritePin(PC_CONTROL2_GPIO_Port, PC_CONTROL2_Pin, GPIO_PIN_RESET); // ONOFF
//			break;
//			case PC_POWERON_DELAY+2:
//			HAL_GPIO_WritePin(PC_CONTROL1_GPIO_Port, PC_CONTROL1_Pin, GPIO_PIN_SET); // RESET
//			HAL_GPIO_WritePin(PC_CONTROL2_GPIO_Port, PC_CONTROL2_Pin, GPIO_PIN_SET); // ONOFF
//			break;
//			}
		}
//	}
	//HAL_GPIO_WritePin(GPIOE, RES_PC_2_Pin, GPIO_PIN_SET); // ONOFF
	xTimerStart(SilenceTimer, 50);
  /* USER CODE END tSilence_func */
}

/* tTechCommTImer_callback function */
void tTechCommTImer_callback(void *argument)
{
  /* USER CODE BEGIN tTechCommTImer_callback */

  /* USER CODE END tTechCommTImer_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

