#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>
#include <stdbool.h>

#include "robot.h"
#include "messages.h"

extern struct robotState_s 						rState;
extern struct robotThrusters_s 					rThrusters[THRUSTERS_NUMBER];
extern struct robotSensors_s 					rSensors;
extern struct robotPc_s 						rComputer;
extern struct robotJoystickSpeed_s 				rJoySpeed;
extern struct robotPositionMovement_s 			rPosMov;
extern struct robotDevices_s 					rDevice[DEV_AMOUNT];
extern struct RobotLogicDevices_s 				rLogicDevice[LOGDEV_AMOUNT];
extern struct robotStabilizationConstants_s 	rStabConstants[STABILIZATION_AMOUNT];
extern struct robotStabilizationState_s 		rStabState[STABILIZATION_AMOUNT];

extern uint8_t ShoreRequestBuffer[REQUEST_CONFIG_LENGTH];
extern uint8_t ShoreResponseBuffer[SHORE_CONFIG_RESPONSE_LENGTH];

extern uint8_t ImuRequestBuffer[IMU_REQUEST_LENGTH];
extern uint8_t ImuResetRequestBuffer[IMU_REQUEST_LENGTH_AC];
extern uint8_t ImuResponseBuffer[IMU_RESPONSE_LENGTH];

extern uint8_t ThrustersRequestBuffer[THRUSTERS_REQUEST_LENGTH];
extern uint8_t ThrustersResponseBuffer[THRUSTERS_NUMBER][THRUSTERS_RESPONSE_LENGTH];

extern uint8_t DevicesRequestBuffer[DEVICES_REQUEST_LENGTH];
extern uint8_t DevicesResponseBuffer[DEVICES_NUMBER][DEVICES_RESPONSE_LENGTH];

extern uint8_t PressureResponseBuffer[PRESSURE_SENSOR_SIZE];

extern uint8_t ContourSelected;

#endif
