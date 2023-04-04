/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2023 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
osThreadId tLedBlinkingTaskHandle;
uint32_t tLedBlinkingTaskBuffer[ 128 ];
osStaticThreadDef_t tLedBlinkingTaskControlBlock;
osThreadId tVmaCommTaskHandle;
uint32_t tVmaCommTaskBuffer[ 128 ];
osStaticThreadDef_t tVmaCommTaskControlBlock;
osThreadId tImuCommTaskHandle;
uint32_t tImuCommTaskBuffer[ 128 ];
osStaticThreadDef_t tImuCommTaskControlBlock;
osThreadId tStabilizationTaskHandle;
uint32_t tStabilizationTaskBuffer[ 128 ];
osStaticThreadDef_t tStabilizationTaskControlBlock;
osThreadId tDevCommTaskHandle;
uint32_t tDevCommTaskBuffer[ 128 ];
osStaticThreadDef_t tDevCommTaskControlBlock;
osThreadId tSensCommTaskHandle;
uint32_t tSensCommTaskBuffer[ 128 ];
osStaticThreadDef_t tSensCommTaskControlBlock;
osThreadId tPcCommTaskHandle;
uint32_t tPcCommTaskBuffer[ 128 ];
osStaticThreadDef_t tPcCommTaskControlBlock;
osTimerId tUartTimerHandle;
osTimerId tSilenceHandle;
osMutexId mutDataHandle;
osStaticMutexDef_t mutDataControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void nullIntArray(uint8_t *array, uint8_t size);
/* USER CODE END FunctionPrototypes */

void func_tLedBlinkingTask(void const * argument);
void func_tVmaCommTask(void const * argument);
void func_tImuCommTask(void const * argument);
void func_tStabilizationTask(void const * argument);
void func_tDevCommTask(void const * argument);
void func_tSensCommTask(void const * argument);
void func_tPcCommTask(void const * argument);
void func_tUartTimer(void const * argument);
void tSilence_func(void const * argument);

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
    uartBusesInit();
    variableInit();
    stabilizationInit();





  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of mutData */
  osMutexStaticDef(mutData, &mutDataControlBlock);
  mutDataHandle = osMutexCreate(osMutex(mutData));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of tUartTimer */
  osTimerDef(tUartTimer, func_tUartTimer);
  tUartTimerHandle = osTimerCreate(osTimer(tUartTimer), osTimerOnce, NULL);

  /* definition and creation of tSilence */
  osTimerDef(tSilence, tSilence_func);
  tSilenceHandle = osTimerCreate(osTimer(tSilence), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  SilenceTimer = xTimerCreate("silence", DELAY_SILENCE/portTICK_RATE_MS, pdFALSE, 0, (TimerCallbackFunction_t) tSilence_func);
  UARTTimer = xTimerCreate("timer", DELAY_TIMER_TASK/portTICK_RATE_MS, pdFALSE, 0, (TimerCallbackFunction_t) func_tUartTimer);

  xTimerStart(SilenceTimer, 10);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of tLedBlinkingTask */
  osThreadStaticDef(tLedBlinkingTask, func_tLedBlinkingTask, osPriorityLow, 0, 128, tLedBlinkingTaskBuffer, &tLedBlinkingTaskControlBlock);
  tLedBlinkingTaskHandle = osThreadCreate(osThread(tLedBlinkingTask), NULL);

  /* definition and creation of tVmaCommTask */
  osThreadStaticDef(tVmaCommTask, func_tVmaCommTask, osPriorityNormal, 0, 128, tVmaCommTaskBuffer, &tVmaCommTaskControlBlock);
  tVmaCommTaskHandle = osThreadCreate(osThread(tVmaCommTask), NULL);

  /* definition and creation of tImuCommTask */
  osThreadStaticDef(tImuCommTask, func_tImuCommTask, osPriorityBelowNormal, 0, 128, tImuCommTaskBuffer, &tImuCommTaskControlBlock);
  tImuCommTaskHandle = osThreadCreate(osThread(tImuCommTask), NULL);

  /* definition and creation of tStabilizationTask */
  osThreadStaticDef(tStabilizationTask, func_tStabilizationTask, osPriorityBelowNormal, 0, 128, tStabilizationTaskBuffer, &tStabilizationTaskControlBlock);
  tStabilizationTaskHandle = osThreadCreate(osThread(tStabilizationTask), NULL);

  /* definition and creation of tDevCommTask */
  osThreadStaticDef(tDevCommTask, func_tDevCommTask, osPriorityBelowNormal, 0, 128, tDevCommTaskBuffer, &tDevCommTaskControlBlock);
  tDevCommTaskHandle = osThreadCreate(osThread(tDevCommTask), NULL);

  /* definition and creation of tSensCommTask */
//  osThreadStaticDef(tSensCommTask, func_tSensCommTask, osPriorityBelowNormal, 0, 128, tSensCommTaskBuffer, &tSensCommTaskControlBlock);
//  tSensCommTaskHandle = osThreadCreate(osThread(tSensCommTask), NULL);

  /* definition and creation of tPcCommTask */
  osThreadStaticDef(tPcCommTask, func_tPcCommTask, osPriorityLow, 0, 128, tPcCommTaskBuffer, &tPcCommTaskControlBlock);
  tPcCommTaskHandle = osThreadCreate(osThread(tPcCommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer, 1);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_func_tLedBlinkingTask */
/**
  * @brief  Function implementing the tLedBlinkingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func_tLedBlinkingTask */
void func_tLedBlinkingTask(void const * argument)
{
  /* USER CODE BEGIN func_tLedBlinkingTask */
    uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
        HAL_GPIO_TogglePin(GPIOB, led1_Pin);
        osDelayUntil(&sysTime, DELAY_LED_TASK);
        HAL_GPIO_TogglePin(GPIOB, led2_Pin);
        osDelayUntil(&sysTime, DELAY_LED_TASK);
        HAL_GPIO_TogglePin(GPIOB, led3_Pin);
        osDelayUntil(&sysTime, DELAY_LED_TASK);
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
void func_tVmaCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tVmaCommTask */
	uint32_t sysTime = osKernelSysTick();
	uint8_t transaction = 0;
	HAL_GPIO_WritePin(RE_DE_GPIO_Port,RE_DE_Pin,GPIO_PIN_SET);
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
		osDelayUntil(&sysTime, DELAY_THRUSTERS_TASK);
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
void func_tImuCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tImuCommTask */
  uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  	if(rSensors.resetIMU) {
			uartBus[IMU_UART].txBuffer = ImuResetRequestBuffer;
			uartBus[IMU_UART].txLength = IMU_REQUEST_LENGTH_AC;
	  		transmitPackage(&uartBus[IMU_UART], false);

	  		osDelayUntil(&sysTime, DELAY_IMU_TASK);
			uartBus[IMU_UART].txBuffer = ImuRequestBuffer;
			uartBus[IMU_UART].txLength = IMU_REQUEST_LENGTH;
	  		transmitPackage(&uartBus[IMU_UART], false);

	  		rSensors.pressure_null = rSensors.pressure;
	  		rSensors.resetIMU = false;
	  	}
	  	else {

	  		uartBus[IMU_UART].rxBuffer = ImuResponseBuffer;
	  		uartBus[IMU_UART].rxLength = IMU_RESPONSE_LENGTH;

	  		HAL_UART_Receive_IT(uartBus[IMU_UART].huart, uartBus[IMU_UART].rxBuffer, uartBus[IMU_UART].rxLength);
	  		osDelayUntil(&sysTime, DELAY_IMU_TASK);

	  		//if(transmitAndReceive(&uartBus[IMU_UART], false)) {
	  			if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_IMU_TASK) == pdTRUE) {
	  				ImuReceive(ImuResponseBuffer);
	  				xSemaphoreGive(mutDataHandle);
	  			}
	  		//}

	  	}

	  	osDelayUntil(&sysTime, DELAY_IMU_TASK);
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
void func_tStabilizationTask(void const * argument)
{
  /* USER CODE BEGIN func_tStabilizationTask */
	uint32_t sysTime = osKernelSysTick();
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

		osDelayUntil(&sysTime, DELAY_STABILIZATION_TASK);
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
void func_tDevCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tDevCommTask */
    uint32_t sysTime = osKernelSysTick();
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
        osDelayUntil(&sysTime, DELAY_DEVICES_TASK);
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
void func_tSensCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tSensCommTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  receiveI2cPackageDMA(DEV_I2C, SENSORS_PRESSURE_ADDR, PressureResponseBuffer, PRESSURE_SENSOR_SIZE);
	  osDelayUntil(&sysTime, DELAY_SENSOR_TASK);
	  if(xSemaphoreTake(mutDataHandle, (TickType_t) DELAY_SENSOR_TASK) == pdTRUE) {
	  	  SensorsResponseUpdate(PressureResponseBuffer, DEV_I2C);
	  	  xSemaphoreGive(mutDataHandle);
	  }
	  osDelayUntil(&sysTime, DELAY_SENSOR_TASK);
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
void func_tPcCommTask(void const * argument)
{
  /* USER CODE BEGIN func_tPcCommTask */
	uint32_t sysTime = osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
	  osDelayUntil(&sysTime, DELAY_PC_TASK);
  }
  /* USER CODE END func_tPcCommTask */
}

/* func_tUartTimer function */
void func_tUartTimer(void const * argument)
{
  /* USER CODE BEGIN func_tUartTimer */
	if (uartBus[SHORE_UART].packageReceived) {
		bool package = true;
		if(xSemaphoreTake(mutDataHandle, (TickType_t) WAITING_TIMER) == pdTRUE) {
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
void tSilence_func(void const * argument)
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
