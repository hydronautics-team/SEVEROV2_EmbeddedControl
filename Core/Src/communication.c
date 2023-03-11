#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "math.h"

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
	uartBus[SHORE_UART].huart = &huart1; // Link to huart will be set before receiving
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
	uartBus[THRUSTERS_UART].huart = &huart2;
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
	uartBus[DEVICES_UART].huart = &huart3;
	uartBus[DEVICES_UART].rxBuffer = 0; // Receive bugger will be set before receive
	uartBus[DEVICES_UART].txBuffer = 0; // Transmit bugger will be set before transmit
	uartBus[DEVICES_UART].rxLength = DEVICES_REQUEST_LENGTH;
	uartBus[DEVICES_UART].txLength = DEVICES_RESPONSE_LENGTH;
	uartBus[DEVICES_UART].brokenRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].timeoutRxTolerance = 0; // There is no special event on this bus
	uartBus[DEVICES_UART].receiveTimeout = 100;
	uartBus[DEVICES_UART].transmitTimeout = 100;
	uartBus[DEVICES_UART].txrxType = TXRX_DMA;

//	// IMU UART configuration
//	uartBus[IMU_UART].huart = &huart4;
//	uartBus[IMU_UART].rxBuffer = ImuResponseBuffer;
//	uartBus[IMU_UART].txBuffer = 0; // Buffer will be set before transmit
//	uartBus[IMU_UART].rxLength = 0; // Receive length will be set before transmit
//	uartBus[IMU_UART].txLength = 0; // Transmit length will be set before transmit
//	uartBus[IMU_UART].brokenRxTolerance = 0; // There is no special event on this bus
//	uartBus[IMU_UART].timeoutRxTolerance = 0; // There is no special event on this bus
//	uartBus[IMU_UART].receiveTimeout = 100;
//	uartBus[IMU_UART].transmitTimeout = 100;
//	uartBus[IMU_UART].txrxType = TXRX_IT;

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

        rJoySpeed.march = req.march;
        rJoySpeed.lag = req.lag;
        rJoySpeed.depth = req.depth;
        rJoySpeed.roll = req.roll;
        rJoySpeed.pitch = req.pitch;
        rJoySpeed.yaw = req.yaw;

        rDevice[GRAB].force = req.grab;
        rDevice[GRAB_ROTATION].force  = req.grab_rotate;
        rDevice[TILT].force = req.tilt;
        rDevice[DEV1].force = req.dev1;
        rDevice[DEV2].force = req.dev2;
        rDevice[DEV3].force = req.dev3;

        rSensors.resetIMU = PickBit(req.stabilize_flags, SHORE_BIT_RESET_IMU);
		rState.thrustersOn = PickBit(req.stabilize_flags, SHORE_BIT_ON_THRUSTERS);

        bool wasEnabled = rStabConstants[STAB_ROLL].enable;
        rStabConstants[STAB_ROLL].enable = PickBit(req.flags, SHORE_BIT_STABILIZE_ROLL);
        if(wasEnabled == false && rStabConstants[STAB_ROLL].enable == true) {
        	stabilizationStart(STAB_ROLL);
        }

        bool wasEnabled = rStabConstants[STAB_YAW].enable;
        rStabConstants[STAB_YAW].enable = PickBit(req.flags, SHORE_BIT_STABILIZE_YAW);
        if(wasEnabled == false && rStabConstants[STAB_YAW].enable == true) {
        	stabilizationStart(STAB_YAW);
        }

        wasEnabled = rStabConstants[STAB_PITCH].enable;
        rStabConstants[STAB_PITCH].enable = PickBit(req.flags, SHORE_BIT_STABILIZE_PITCH);
        if(wasEnabled == false && rStabConstants[STAB_PITCH].enable == true) {
        	stabilizationStart(STAB_PITCH);
        }

        wasEnabled = rStabConstants[STAB_DEPTH].enable;
        rStabConstants[STAB_DEPTH].enable = PickBit(req.flags, SHORE_BIT_STABILIZE_DEPTH);
        if(wasEnabled == false && rStabConstants[STAB_DEPTH].enable == true) {
        	stabilizationStart(STAB_DEPTH);
        }

        formThrustVectors();

        ++uartBus[SHORE_UART].successRxCounter;
    }
    else {
    	++uartBus[SHORE_UART].brokenRxCounter;

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

		if(rState.contourSelected != req.contour) {
			for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
				rStabConstants[i].enable = false;
			}
			rState.contourSelected = req.contour;
			stabilizationStart(req.contour);
		}

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
			else {
				rThrusters[req.id].desiredSpeed = req.velocity;
				rThrusters[req.id].address = req.address;
				rThrusters[req.id].kForward = req.kForward;
				rThrusters[req.id].kBackward = req.kBackward;
				rThrusters[req.id].sForward = req.sForward;
				rThrusters[req.id].sBackward = req.sBackward;
				rThrusters[req.id].inverse = req.reverse;
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
    res.yaw = rSensors.yaw;
	res.depth = rSensors.pressure

    res.rollSpeed = rSensors.rollSpeed;
    res.pitchSpeed = rSensors.pitchSpeed;
    res.yawSpeed = rSensors.yawSpeed;

    memcpy((void*)responseBuf, (void*)&res, SHORE_RESPONSE_LENGTH);
    AddCrc16Checksumm(responseBuf, SHORE_RESPONSE_LENGTH);
}

void ShoreConfigResponse(uint8_t *responseBuf)
{
	struct shoreConfigResponse_s res;

	res.roll = rSensors.roll;
	res.pitch = rSensors.pitch;
	res.yaw = rSensors.yaw;
	res.depth = rSensors.pressure

	res.rollSpeed = rSensors.rollSpeed;
	res.pitchSpeed = rSensors.pitchSpeed;
	res.yawSpeed = rSensors.yawSpeed;

	res.inputSignal = *rStabState[rState.contourSelected].inputSignal;
	res.speedSignal = *rStabState[rState.contourSelected].speedSignal;
	res.posSignal = *rStabState[rState.contourSelected].posSignal;

	res.joyUnitCaste 	= rStabState[rState.contourSelected].joyUnitCasted;
	res.posError 		= rStabState[rState.contourSelected].posError;
	res.joy_iValue 		= rStabState[rState.contourSelected].joy_iValue;
	res.speedError 		= rStabState[rState.contourSelected].speedError;
	res.dynSummator 	= rStabState[rState.contourSelected].dynSummator;
	res.pidValue 		= rStabState[rState.contourSelected].pidValue;
	res.posErrorAmp 	= rStabState[rState.contourSelected].posErrorAmp;
	res.speedFiltere 	= rStabState[rState.contourSelected].speedFiltered;
	res.posFiltered 	= rStabState[rState.contourSelected].posFiltered;
	res.pid_iValue 		= rStabState[rState.contourSelected].pid_iValue;
	res.pid_pValue 		= rStabState[rState.contourSelected].pid_pValue;
	res.outputSignal 	= rStabState[rState.contourSelected].outputSignal;

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

void ImuReceive(uint8_t *ReceiveBuf)
{

    for(uint8_t i = 0; i < IMU_CHECKSUMS; ++i) {
        if(!IsChecksum16bSCorrect(&ReceiveBuf[i*IMU_RESPONSE_LENGTH], IMU_RESPONSE_LENGTH)) {
            ++uartBus[IMU_UART].brokenRxCounter;
            return;
        }
    }

    rSensors.rollSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_Y])) * 0.0610352;
    rSensors.pitchSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_X])) * 0.0610352;
    rSensors.yawSpeed = (float) (MergeBytes(&ReceiveBuf[GYRO_PROC_Z])) * 0.0610352;

    rSensors.accelX = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_X])) * 0.0109863;
    rSensors.accelY = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_Y])) * 0.0109863;
    rSensors.accelZ = (float) (MergeBytes(&ReceiveBuf[ACCEL_PROC_Z])) * 0.0109863;

    rSensors.magX = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_X])) * 0.000183105;
    rSensors.magY = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_Y])) * 0.000183105;
    rSensors.magZ = (float) (MergeBytes(&ReceiveBuf[MAG_PROC_Z])) * 0.000183105;

    rSensors.quatA = (float) (MergeBytes(&ReceiveBuf[QUAT_A])) * 0.0000335693;
    rSensors.quatB = (float) (MergeBytes(&ReceiveBuf[QUAT_B])) * 0.0000335693;
    rSensors.quatC = (float) (MergeBytes(&ReceiveBuf[QUAT_C])) * 0.0000335693;
    rSensors.quatD = (float) (MergeBytes(&ReceiveBuf[QUAT_D])) * 0.0000335693;

	float diffTime = fromTickToMs(xTaskGetTickCount() - rSensors.LastTick) / 1000.0f;
    rSensors.LastTick = xTaskGetTickCount();
    rSensors.yaw += (rSensors.yawSpeed * diffTime);

    //rSensors.yaw = (float) (MergeBytes(&ReceiveBuf[EULER_PSI])) * 0.0109863;
    rSensors.roll =  0;//asin(rSensors.accelX/62)*180/3.14;
    rSensors.pitch =  (float) asin(rSensors.accelY/62)*180/3.14;//(float) (MergeBytes(&ReceiveBuf[EULER_PHI])) * 0.0109863;

    ++uartBus[IMU_UART].successRxCounter;
}
