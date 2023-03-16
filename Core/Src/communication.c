#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "math.h"
#include "can.h"

#include "communication.h"
#include "global.h"
#include "messages.h"
#include "checksum.h"
#include "robot.h"
#include "stabilization.h"
#include "flash.h"
#include "thrusters.h"
#include "timers.h"

#define PACKAGE_TOLLERANCE 	20
#define THRUSTER_FILTERS_NUMBER 2

enum thrusterContours {
	CONTOUR_MARCH = 0,
	CONTOUR_LAG
};

extern TimerHandle_t UARTTimer;

struct uartBus_s uartBus[UART_NUMBER];

uint8_t VMAbrokenRxTolerance = 0;

const uint16_t ShoreLength[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_LENGTH, REQUEST_CONFIG_LENGTH, SHORE_REQUEST_DIRECT_LENGTH};
const uint8_t ShoreCodes[SHORE_REQUEST_MODES_NUMBER] = {SHORE_REQUEST_CODE, REQUEST_CONFIG_CODE, DIRECT_REQUEST_CODE};

uint16_t counterRx = 0;

bool i2c1PackageTransmit = false;
bool i2c1PackageReceived = false;


void variableInit()
{
	rComputer.reset = 0;

	rState.cameraNum = 0;
	rState.contourSelected = 0;
	rState.flash = 0;
	rState.operationMode = 0;
	rState.pcCounter = 0;
	rState.lag_error = 0;

	rSensors.yaw = 0;
	rSensors.raw_yaw = 0;
	rSensors.roll =  0;
	rSensors.pitch =  0;

	rSensors.old_yaw = 0;
	rSensors.spins = 0;

	rSensors.pressure = 0;
	rSensors.pressure_null = 0;

	rSensors.rollSpeed = 0;
	rSensors.pitchSpeed = 0;
	rSensors.yawSpeed = 0;

	rSensors.accelX = 0;
	rSensors.accelY = 0;
	rSensors.accelZ = 0;

	rSensors.magX = 0;
	rSensors.magY = 0;
	rSensors.magZ = 0;

	rSensors.quatA = 0;
	rSensors.quatB = 0;
	rSensors.quatC = 0;
	rSensors.quatD = 0;

    rDevice[DEV1].address = 0x03;
    rDevice[DEV2].address = 0x05;
    rDevice[GRAB].address = 0x02;
    rDevice[GRAB_ROTATION].address = 0x06;
    rDevice[TILT].address = 0x01;

	rSensors.resetIMU = true;

	thrustersInit();

	// Flash reading
	struct flashConfiguration_s config;
	flashReadSettings(&config);
	flashReadStructure(&config);

	// Thrusters initialization
	if(rState.flash) {
		return;
	}
}

void uartBusesInit()
{
	// Shore UART configuration
	uartBus[SHORE_UART].huart = &huart3; // Link to huart will be set before receiving
	uartBus[SHORE_UART].rxBuffer = ShoreRequestBuffer;
	uartBus[SHORE_UART].txBuffer = ShoreResponseBuffer;
	uartBus[SHORE_UART].rxLength = 0; // Length of the received message will be determined when first byte will be received
	uartBus[SHORE_UART].txLength = 0; // Length of the transmitted message will be determined before transmit
	uartBus[SHORE_UART].brokenRxTolerance = 20;
	uartBus[SHORE_UART].timeoutRxTolerance = 500;
	uartBus[SHORE_UART].receiveTimeout = 200;
	uartBus[SHORE_UART].transmitTimeout = 200;
	uartBus[SHORE_UART].txrxType = TXRX_IT;

	// Thrusters UART configuration
	uartBus[THRUSTERS_UART].huart = &huart1;
	uartBus[THRUSTERS_UART].rxBuffer = 0; // Receive bugger will be set before receive
	uartBus[THRUSTERS_UART].txBuffer = 0; // Transmit bugger will be set before transmit
	uartBus[THRUSTERS_UART].rxLength = 0; // Receive length will be set before transmit
	uartBus[THRUSTERS_UART].txLength = 0; // Transmit length will be set before transmit
	uartBus[THRUSTERS_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[THRUSTERS_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[THRUSTERS_UART].receiveTimeout = 100;
	uartBus[THRUSTERS_UART].transmitTimeout = 100;
	uartBus[THRUSTERS_UART].txrxType = TXRX_DMA;

	// Devices UART configuration
	uartBus[DEVICES_UART].huart = &huart1;
	uartBus[DEVICES_UART].rxBuffer = 0; // Receive bugger will be set before receive
	uartBus[DEVICES_UART].txBuffer = 0; // Transmit bugger will be set before transmit
	uartBus[DEVICES_UART].rxLength = DEVICES_REQUEST_LENGTH;
	uartBus[DEVICES_UART].txLength = DEVICES_RESPONSE_LENGTH;
	uartBus[DEVICES_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].receiveTimeout = 100;
	uartBus[DEVICES_UART].transmitTimeout = 100;
	uartBus[DEVICES_UART].txrxType = TXRX_DMA;

	// IMU UART configuration
	uartBus[IMU_UART].huart = &huart2;
	uartBus[IMU_UART].rxBuffer = ImuResponseBuffer;
	uartBus[IMU_UART].txBuffer = 0; // Buffer will be set before transmit
	uartBus[IMU_UART].rxLength = 0; // Receive length will be set before transmit
	uartBus[IMU_UART].txLength = 0; // Transmit length will be set before transmit
	uartBus[IMU_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[IMU_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[IMU_UART].receiveTimeout = 100;
	uartBus[IMU_UART].transmitTimeout = 100;
	uartBus[IMU_UART].txrxType = TXRX_IT;

	for(uint8_t i=0; i<UART_NUMBER; i++) {
		uartBus[i].packageReceived = false;
		uartBus[i].packageTransmitted = false;
		uartBus[i].successRxCounter = 0;
		uartBus[i].brokenRxCounter = 0;
		uartBus[i].outdatedRxCounter = 0;
		uartBus[i].timeoutCounter = 0;
		uartBus[i].lastMessage = 0;
	}
}

bool transmitPackage(struct uartBus_s *bus, bool isrMode)
{
    bus->packageTransmitted = false;

    HAL_UART_AbortTransmit_IT(bus->huart);
    switch(bus->txrxType) {
        case TXRX_DMA:
            HAL_UART_Transmit_DMA(bus->huart, bus->txBuffer, bus->txLength);
            break;
        case TXRX_IT:
        	HAL_UART_Transmit_IT(bus->huart, bus->txBuffer, bus->txLength);
            break;
        default:
            return false;
    }

    bus->timeoutCounter = fromTickToMs(xTaskGetTickCount());
    while (!bus->packageTransmitted && !isrMode) {
    	if(fromTickToMs(xTaskGetTickCount()) - bus->timeoutCounter > bus->transmitTimeout) {
    		return false;
    	}
    	osDelay(DELAY_UART_TIMEOUT);
    }
    return true;
}

bool receivePackage(struct uartBus_s *bus, bool isrMode)
{
	bus->packageReceived = false;

	HAL_UART_AbortReceive_IT(bus->huart);
	switch(bus->txrxType) {
		case TXRX_DMA:
			HAL_UART_Receive_DMA(bus->huart, bus->rxBuffer, bus->rxLength);
			break;
		case TXRX_IT:
			HAL_UART_Receive_IT(bus->huart, bus->rxBuffer, bus->rxLength);
			break;
		default:
			return false;
	}

	bus->timeoutCounter = HAL_GetTick();
	while (!bus->packageReceived && !isrMode) {
		if(HAL_GetTick() - bus->timeoutCounter > bus->receiveTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

bool transmitAndReceive(struct uartBus_s *bus, bool isrMode)
{
	bus->packageReceived = false;
	bus->packageTransmitted = false;

	HAL_UART_AbortReceive_IT(bus->huart);
	HAL_UART_AbortTransmit_IT(bus->huart);
	switch(bus->txrxType) {
		case TXRX_DMA:
			HAL_UART_Receive_DMA(bus->huart, bus->rxBuffer, bus->rxLength);
			HAL_UART_Transmit_DMA(bus->huart, bus->txBuffer, bus->txLength);
			break;
		case TXRX_IT:
			HAL_UART_Receive_IT(bus->huart, bus->rxBuffer, bus->rxLength);
			HAL_UART_Transmit_IT(bus->huart, bus->txBuffer, bus->txLength);
			break;
		default:
			return false;
	}

	bus->timeoutCounter = fromTickToMs(xTaskGetTickCount());
	while (!bus->packageTransmitted && !bus->packageReceived && !isrMode) {
		if(fromTickToMs(xTaskGetTickCount()) - bus->timeoutCounter > bus->transmitTimeout) {
			return false;
		}
		osDelay(DELAY_UART_TIMEOUT);
	}
	return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == uartBus[SHORE_UART].huart) {
		uartBus[SHORE_UART].packageTransmitted = true;
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			bus->packageTransmitted = true;
			break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == uartBus[SHORE_UART].huart) {
		ShoreReceive();
		return;
	}

	struct uartBus_s *bus = 0;
	for(uint8_t i=0; i<UART_NUMBER; i++) {
		if(uartBus[i].huart == huart) {
			bus = &uartBus[i];
			bus->packageReceived = true;
			bus->lastMessage = fromTickToMs(xTaskGetTickCount());
			break;
		}
	}
}

bool receiveI2cPackageDMA (uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
	float timeBegin = fromTickToMs(xTaskGetTickCount());
	i2c1PackageReceived = false;
	switch(I2C) {
	case DEV_I2C:
		HAL_I2C_Master_Receive_IT(&hi2c1, addr>>1, buf, length);
		while (!i2c1PackageReceived) {
			if(fromTickToMs(xTaskGetTickCount()) - timeBegin > WAITING_SENSORS) {
				//HAL_I2C_Master_Abort_IT(&hi2c2, addr>>1);
				HAL_I2C_Init(&hi2c1);
				return false;
			}
			osDelay(DELAY_SENSOR_TASK);
		}
		break;
	}
	return true;
}


void transmitI2cPackageDMA(uint8_t I2C, uint16_t addr, uint8_t *buf, uint8_t length)
{
	TickType_t timeBegin = xTaskGetTickCount();
	i2c1PackageTransmit = false;
	switch(I2C) {
	case DEV_I2C:
		HAL_I2C_Master_Transmit_IT(&hi2c1, addr>>1, buf, length);
		while (!i2c1PackageTransmit && xTaskGetTickCount() - timeBegin < WAITING_SENSORS) {
			osDelay(DELAY_SENSOR_TASK);
		}
		break;
	}
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c1PackageReceived = true;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1) {
		i2c1PackageTransmit = true;
	}
}


void SensorsResponseUpdate(uint8_t *buf, uint8_t Sensor_id)
{
	switch(Sensor_id) {
	case DEV_I2C:
		if(IsChecksumm8bCorrect(buf, PRESSURE_SENSOR_SIZE)) {
			struct pressureResponse_s res;
			memcpy((void*)&res, (void*)buf, DEVICES_RESPONSE_LENGTH);
			if(res.code == 0xAA) {
				rSensors.pressure = res.value;//(9.124*res.value - 3.177) - rSensors.pressure_null;
				rSensors.velocity_pressure = res.v_value;
			}
		}
		break;
	}
}

void ShoreReceive()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if(counterRx == 0) {
		for(uint8_t i=0; i<SHORE_REQUEST_MODES_NUMBER; ++i) {
			if(uartBus[SHORE_UART].rxBuffer[0] == ShoreCodes[i]) {
				counterRx = 1;
				uartBus[SHORE_UART].rxLength = ShoreLength[i]-1;
				HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer+1, uartBus[SHORE_UART].rxLength);
				xTimerStartFromISR(UARTTimer, &xHigherPriorityTaskWoken);
				break;
			}

			if(i == SHORE_REQUEST_MODES_NUMBER-1) {
				HAL_UART_Receive_IT(uartBus[SHORE_UART].huart, uartBus[SHORE_UART].rxBuffer, 1);
			}
		}
	}
	else if(counterRx == 1) {
		uartBus[SHORE_UART].packageReceived = true;
		uartBus[SHORE_UART].lastMessage = fromTickToMs(xTaskGetTickCount());
		counterRx = 2;
	}

	if (xHigherPriorityTaskWoken == pdTRUE) {
		xHigherPriorityTaskWoken = pdFALSE;
		taskYIELD();
	}
}

void DevicesRequestUpdate(uint8_t *buf, uint8_t dev)
{
	struct devicesRequest_s req;

    req.AA1 = 0xAA;
    req.AA2 = 0xAA;
    req.address = rDevice[dev].address;
    req.setting = rDevice[dev].settings;
    req.velocity1 = 0;
    req.velocity2 = rDevice[dev].force;

    if(dev == GRAB) {
    	req.velocity1 = rDevice[GRAB_ROTATION].force;
    	req.velocity2 = rDevice[GRAB].force;
    }

//    if(dev == TILT) {
//    	switch(rLogicDevice[LOGDEV_LIFTER].state) {
//    	case LOGDEV_FORWARD:
//    		req.velocity2 = 60;
//    		break;
//    	case LOGDEV_BACKWARD:
//    		req.velocity2 = -60;
//    		break;
//    	case LOGDEV_FORWARD_SAT:
//    		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_NULL;
//    		req.velocity2 = 0;
//    		break;
//    	case LOGDEV_BACKWARD_SAT:
//    		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_NULL;
//    		req.velocity2 = 0;
//    	}
//    }

//    if(rLogicDevice[LOGDEV_LIFTER].state == LOGDEV_FORWARD_SAT) {
//    	if(req.velocity2 > 0) {
//    		req.velocity2 = 0;
//    	}
//    	else if(req.velocity2 < 0) {
//    		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_NULL;
//    	}
//    }
//    else if(rLogicDevice[LOGDEV_LIFTER].state == LOGDEV_BACKWARD_SAT) {
//    	if(req.velocity2 < 0) {
//    		req.velocity2 = 0;
//    	}
//    	else if(req.velocity2 > 0) {
//    		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_NULL;
//    	}
//    }


    memcpy((void*)buf, (void*)&req, DEVICES_REQUEST_LENGTH);
    AddChecksumm8b(buf, DEVICES_REQUEST_LENGTH);
}

void DevicesResponseUpdate(uint8_t *buf, uint8_t dev)
{
    if(IsChecksumm8bCorrect(buf, DEVICES_RESPONSE_LENGTH)) {
    	struct devicesResponse_s res;
    	memcpy((void*)&res, (void*)buf, DEVICES_RESPONSE_LENGTH);

        rDevice[dev].current = res.current1;
        rDevice[dev].velocity1 = res.velocity1;
        rDevice[dev].velocity2 = res.velocity2;

        if(rDevice[DEV2].velocity1 == 0x00 && dev == DEV2) {
        	rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_FORWARD_SAT;
        }
        else if(rDevice[DEV2].velocity2 == 0x00 && dev == DEV2) {
        	rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_BACKWARD_SAT;
        }
        // TODO make errors work pls
        //writeBit(&(robot->device[dev].errors), res.errors, AGAR);

        ++uartBus[DEVICES_UART].successRxCounter;
    }
    else {
    	++uartBus[DEVICES_UART].brokenRxCounter;
    }
}

void ShoreRequest(uint8_t *requestBuf)
{
    if (IsCrc16ChecksummCorrect(requestBuf, SHORE_REQUEST_LENGTH)) {
    	struct shoreRequest_s req;
    	memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_LENGTH);

    	uint8_t tempCameraNum = 0;

        rJoySpeed.march = req.march;
        rJoySpeed.lag = req.lag;
        rJoySpeed.depth = req.depth;
        rJoySpeed.roll = req.roll;
        rJoySpeed.pitch = req.pitch;
        rJoySpeed.yaw = req.yaw;

        rDevice[GRAB].force = req.grab;
        if (rDevice[GRAB].force < -127) {
            rDevice[GRAB].force = -127;
        }
        rDevice[TILT].force = req.tilt;
        if (rDevice[TILT].force < -127) {
        	rDevice[TILT].force = -127;
        }
        rDevice[GRAB_ROTATION].force  = req.grab_rotate;
        if (rDevice[GRAB_ROTATION].force < -127) {
            rDevice[GRAB_ROTATION].force = -127;
        }

        rDevice[DEV1].force = req.dev1;
        rDevice[DEV2].force = req.dev2;

        rState.lag_error = (float) req.lag_error;

        rSensors.resetIMU = PickBit(req.stabilize_flags, SHORE_STABILIZE_IMU_BIT);

        if(PickBit(req.stabilize_flags, SHORE_STABILIZE_SAVE_BIT)) {
        	struct flashConfiguration_s config;
        	flashFillStructure(&config);
        	flashWriteSettings(&config);
        }

        tempCameraNum = req.cameras;

        uint8_t old_reset = rComputer.reset;
        if(old_reset != req.pc_reset) {
            if(req.pc_reset == PC_ON_CODE) {
          //  	HAL_GPIO_WritePin(PC_CONTROL1_GPIO_Port, PC_CONTROL1_Pin, GPIO_PIN_RESET); // RESET
          //  	HAL_GPIO_WritePin(PC_CONTROL2_GPIO_Port, PC_CONTROL2_Pin, GPIO_PIN_RESET); // ONOFF
            }
            else if(req.pc_reset == PC_OFF_CODE) {
           // 	HAL_GPIO_WritePin(PC_CONTROL1_GPIO_Port, PC_CONTROL1_Pin, GPIO_PIN_SET); // RESET
           //	HAL_GPIO_WritePin(PC_CONTROL2_GPIO_Port, PC_CONTROL2_Pin, GPIO_PIN_SET); // ONOFF
            }
        }
        rComputer.reset = req.pc_reset;

        bool wasEnabled = rStabConstants[STAB_YAW].enable;
        rStabConstants[STAB_YAW].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_YAW_BIT);
        if(wasEnabled == false && rStabConstants[STAB_YAW].enable == true) {
        	stabilizationStart(STAB_YAW);
        }

        wasEnabled = rStabConstants[STAB_ROLL].enable;
        rStabConstants[STAB_ROLL].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_ROLL_BIT);
        if(wasEnabled == false && rStabConstants[STAB_ROLL].enable == true) {
        	stabilizationStart(STAB_ROLL);
        }

        wasEnabled = rStabConstants[STAB_PITCH].enable;
        rStabConstants[STAB_PITCH].enable = true; //PickBit(req.stabilize_flags, SHORE_STABILIZE_PITCH_BIT);
        if(wasEnabled == false && rStabConstants[STAB_PITCH].enable == true) {
        	stabilizationStart(STAB_PITCH);
        }

        wasEnabled = rStabConstants[STAB_DEPTH].enable;
        rStabConstants[STAB_DEPTH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT);
        if(wasEnabled == false && rStabConstants[STAB_DEPTH].enable == true) {
        	stabilizationStart(STAB_DEPTH);
        }

        wasEnabled = rStabConstants[STAB_LAG].enable;
        rStabConstants[STAB_LAG].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_LAG_BIT);
        if(wasEnabled == false && rStabConstants[STAB_LAG].enable == true) {
        	stabilizationStart(STAB_LAG);
        }

        wasEnabled = rStabConstants[STAB_MARCH].enable;
        rStabConstants[STAB_MARCH].enable = PickBit(req.stabilize_flags, SHORE_STABILIZE_MARCH_BIT);
        if(wasEnabled == false && rStabConstants[STAB_MARCH].enable == true) {
        	stabilizationStart(STAB_MARCH);
        }

//        wasEnabled = rLogicDevice[LOGDEV_LIFTER].control;
//        rLogicDevice[LOGDEV_LIFTER].control = PickBit(req.stabilize_flags, SHORE_STABILIZE_LOGDEV_BIT);
//        if(wasEnabled != rLogicDevice[LOGDEV_LIFTER].control) {
//        	if(rLogicDevice[LOGDEV_LIFTER].control) {
//        		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_FORWARD;
//        	}
//        	else {
//        		rLogicDevice[LOGDEV_LIFTER].state = LOGDEV_BACKWARD;
//        	}
//        }

//        if(tempCameraNum != rState.cameraNum) {
//        	rState.cameraNum = tempCameraNum;
//        	switch(rState.cameraNum) {
//        	case 0:
//        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_RESET);
//        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_RESET);
//        		break;
//
//        	case 1:
//        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_SET);
//        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_RESET);
//        		break;
//
//        	case 2:
//        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_RESET);
//        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_SET);
//        		break;
//
//        	case 3:
//        		HAL_GPIO_WritePin(GPIOA, CAM1_Pin, GPIO_PIN_SET);
//        		HAL_GPIO_WritePin(GPIOA, CAM2_Pin, GPIO_PIN_SET);
//        		break;
//        	}
//        }

        // TODO tuuuupoooo
        formThrustVectors();

        ++uartBus[SHORE_UART].successRxCounter;
    }
    else {
    	++uartBus[SHORE_UART].brokenRxCounter;

    	//TODO tolerance!
    	/*
        if (brokenRxTolerance == PACKAGE_TOLLERANCE) {
        	robot->i_joySpeed.march = 0;
        	robot->i_joySpeed.lag = 0;
        	robot->i_joySpeed.depth = 0;
        	robot->i_joySpeed.pitch = 0;
        	robot->i_joySpeed.roll = 0;
        	robot->i_joySpeed.yaw = 0;

        	robot->device[LIGHT].force = 0;
        	robot->device[DEV1].force = 0;
        	robot->device[DEV2].force = 0;
        	robot->device[GRAB].force = 0;
        	robot->device[GRAB_ROTATION].force  = 0;
        	robot->device[TILT].force = 0;

        	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i){
        		robot->thrusters[i].desiredSpeed = 0;
        	}

        	brokenRxTolerance = 0;
        }
        */
    }
}

void ShoreConfigRequest(uint8_t *requestBuf)
{
	if(IsCrc16ChecksummCorrect(requestBuf, REQUEST_CONFIG_LENGTH)) {
		struct shoreConfigRequest_s req;
		memcpy((void*)&req, (void*)requestBuf, REQUEST_CONFIG_LENGTH);

		rJoySpeed.march = req.march;
		rJoySpeed.lag = req.lag;
		rJoySpeed.depth = req.depth;
		rJoySpeed.roll = req.roll;
		rJoySpeed.pitch = req.pitch;
		rJoySpeed.yaw = req.yaw;

		rStabConstants[req.contour].pJoyUnitCast = req.pJoyUnitCast;
		rStabConstants[req.contour].pSpeedDyn = req.pSpeedDyn;
		rStabConstants[req.contour].pErrGain = req.pErrGain;

		rStabConstants[req.contour].aFilter[POS_FILTER].T = req.posFilterT;
		rStabConstants[req.contour].aFilter[POS_FILTER].K = req.posFilterK;
		rStabConstants[req.contour].aFilter[SPEED_FILTER].T = req.speedFilterT;
		rStabConstants[req.contour].aFilter[SPEED_FILTER].K = req.speedFilterK;

		rStabConstants[req.contour].pid.pGain = req.pid_pGain;
		rStabConstants[req.contour].pid.iGain = req.pid_iGain;
		rStabConstants[req.contour].pid.iMax = req.pid_iMax;
		rStabConstants[req.contour].pid.iMin = req.pid_iMin;

		rStabConstants[req.contour].pThrustersMin = req.pThrustersMin;
		rStabConstants[req.contour].pThrustersMax = req.pThrustersMax;

		rStabConstants[req.contour].aFilter[THRUSTERS_FILTER].T = req.thrustersFilterT;
		rStabConstants[req.contour].aFilter[THRUSTERS_FILTER].K = req.thrustersFilterK;

		rStabConstants[req.contour].sOutSummatorMax = req.sOutSummatorMax;
		rStabConstants[req.contour].sOutSummatorMin = req.sOutSummatorMin;

		if(rState.contourSelected != req.contour) {
			for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
				rStabConstants[i].enable = false;
			}
			rState.contourSelected = req.contour;
			stabilizationStart(req.contour);
		}

		// TODO tuuuupooo
		formThrustVectors();

		++uartBus[SHORE_UART].successRxCounter;;
	}
	else {
		++uartBus[SHORE_UART].brokenRxCounter;
	}
}

void ShoreDirectRequest(uint8_t *requestBuf)
{
	if(IsCrc16ChecksummCorrect(requestBuf, SHORE_REQUEST_DIRECT_LENGTH)) {
		struct shoreRequestDirect_s req;
		memcpy((void*)&req, (void*)requestBuf, SHORE_REQUEST_DIRECT_LENGTH);

		for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
			rStabConstants[i].enable = false;
		}

		for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
			if(i != req.number) {
				rThrusters[i].desiredSpeed = 0;
			}
			else {
				rThrusters[req.number].desiredSpeed = req.velocity;
				rThrusters[req.number].address = req.id;
				rThrusters[req.number].kForward = req.kForward;
				rThrusters[req.number].kBackward = req.kBackward;
				rThrusters[req.number].sForward = req.sForward;
				rThrusters[req.number].sBackward = req.sBackward;
				rThrusters[req.number].inverse = req.reverse;
			}
		}

		++uartBus[SHORE_UART].successRxCounter;;
	}
	else {
		++uartBus[SHORE_UART].brokenRxCounter;
	}
}

void ShoreResponse(uint8_t *responseBuf)
{
	struct shoreResponse_s res;

    res.roll = rSensors.roll;
    res.pitch = rSensors.pitch;
    res.yaw =  rSensors.yaw;//*rStabState[STAB_YAW].posSignal;//rSensors.yaw;
    res.rollSpeed = rSensors.rollSpeed;
    res.pitchSpeed = rSensors.pitchSpeed;
    res.yawSpeed = rSensors.yawSpeed;

    res.pressure = rSensors.pressure;

    res.vma_errors = 0x55;         //!!!!!TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO do this properly pls
    res.dev_errors = 0;//robot->device.errors;
    res.pc_errors = rComputer.errors;

    memcpy((void*)responseBuf, (void*)&res, SHORE_RESPONSE_LENGTH);
    AddCrc16Checksumm(responseBuf, SHORE_RESPONSE_LENGTH);
}

void ShoreConfigResponse(uint8_t *responseBuf)
{
	struct shoreConfigResponse_s res;

	res.code = REQUEST_CONFIG_CODE;

	res.roll = rSensors.roll;
	res.pitch = rSensors.pitch;
	res.yaw = rSensors.yaw;
	res.raw_yaw = rSensors.raw_yaw;

	res.rollSpeed = rSensors.rollSpeed;
	res.pitchSpeed = rSensors.pitchSpeed;
	res.yawSpeed = rSensors.yawSpeed;

	res.pressure = rSensors.pressure;
	res.in_pressure = 0;

	res.inputSignal = *rStabState[rState.contourSelected].inputSignal;
	res.speedSignal = *rStabState[rState.contourSelected].speedSignal;
	res.posSignal = *rStabState[rState.contourSelected].posSignal;

	res.joyUnitCasted = rStabState[rState.contourSelected].joyUnitCasted;
	res.joy_iValue = rStabState[rState.contourSelected].joy_iValue;
	res.posError = rStabState[rState.contourSelected].posError;
	res.speedError = rStabState[rState.contourSelected].speedError;
	res.dynSummator = rStabState[rState.contourSelected].dynSummator;
	res.pidValue = rStabState[rState.contourSelected].pidValue;
	res.posErrorAmp = rStabState[rState.contourSelected].posErrorAmp;
	res.speedFiltered = rStabState[rState.contourSelected].speedFiltered;
	res.posFiltered = rStabState[rState.contourSelected].posFiltered;

	res.pid_iValue = rStabState[rState.contourSelected].pid_iValue;

	res.thrustersFiltered = rStabState[rState.contourSelected].thrustersFiltered;
	res.outputSignal = rStabState[rState.contourSelected].outputSignal;

	memcpy((void*)responseBuf, (void*)&res, SHORE_CONFIG_RESPONSE_LENGTH);

	AddCrc16Checksumm(responseBuf, SHORE_CONFIG_RESPONSE_LENGTH);
}

void ShoreDirectResponse(uint8_t *responseBuf)
{
	struct shoreResponseDirect_s res;

	res.number = 0xFF;
	res.connection = 0xAA;
	res.current = 0xBB;

    memcpy((void*)responseBuf, (void*)&res, SHORE_DIRECT_RESPONSE_LENGTH);

    AddCrc16Checksumm(responseBuf, SHORE_DIRECT_RESPONSE_LENGTH);
}

int16_t sign(int16_t in)
{
	if(in > 0) {
		return 1;
	}
	else if(in < 0) {
		return -1;
	}
	return 0;
}


// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
unsigned int i;
unsigned short crc = 0;
	for(i=0; i<length; i++){
		crc = (unsigned char)(crc >> 8) | (crc << 8); crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5; }
	return crc;
}

void ImuReceive(uint8_t *ReceiveBuf)
{
	 // Check sync byte
	 if ((ReceiveBuf[0] != 0xFA)&&(ReceiveBuf[1] != 0x01)&&(ReceiveBuf[2] != 0x29)&&(ReceiveBuf[3] != 0x01))
		 return;


//    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
//        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
//            ++uartBus[IMU_UART].brokenRxCounter;
//            return;
//        }
//    }

  	  memcpy(&rSensors.yaw, ReceiveBuf + 12, sizeof(rSensors.yaw));
  	  memcpy(&rSensors.pitch, ReceiveBuf + 16, sizeof(rSensors.pitch));
  	  memcpy(&rSensors.roll, ReceiveBuf + 20, sizeof(rSensors.roll));

  	  memcpy(&rSensors.accelX, ReceiveBuf + 36, sizeof(rSensors.accelX));
  	  memcpy(&rSensors.accelY, ReceiveBuf + 40, sizeof(rSensors.accelY));
  	  memcpy(&rSensors.accelZ, ReceiveBuf + 44, sizeof(rSensors.accelZ));
  	  memcpy(&rSensors.crc, ReceiveBuf + 48, sizeof(rSensors.crc));


//	float diffTime = fromTickToMs(xTaskGetTickCount() - rSensors.LastTick) / 1000.0f;
    rSensors.LastTick = xTaskGetTickCount();


    ++uartBus[IMU_UART].successRxCounter;
}
