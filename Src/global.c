#include "global.h"

struct robotState_s 					rState;
struct robotThrusters_s 				rThrusters[THRUSTERS_NUMBER];
struct robotSensors_s 					rSensors;
struct robotPc_s 						rComputer;
struct robotJoystickSpeed_s 			rJoySpeed;
struct robotPositionMovement_s 			rPosMov;
struct robotDevices_s 					rDevice[DEV_AMOUNT];
struct RobotLogicDevices_s 				rLogicDevice[LOGDEV_AMOUNT];
struct robotStabilizationConstants_s 	rStabConstants[STABILIZATION_AMOUNT];
struct robotStabilizationState_s 		rStabState[STABILIZATION_AMOUNT];


uint8_t ShoreRequestBuffer[REQUEST_CONFIG_LENGTH];
uint8_t ShoreResponseBuffer[SHORE_CONFIG_RESPONSE_LENGTH];

uint8_t ImuRequestBuffer[IMU_REQUEST_LENGTH] = "$VNWRG,75,2,8,01,0108*XX\r\n";
uint8_t ImuStartRequestBuffer[IMU_REQUEST_LENGTH_AC] = "$VNWRG,06,0*XX\r\n";
uint8_t ImuResponseBuffer[IMU_RESPONSE_LENGTH];

uint8_t ThrustersRequestBuffer[THRUSTERS_REQUEST_LENGTH];
uint8_t ThrustersResponseBuffer[THRUSTERS_NUMBER][THRUSTERS_RESPONSE_LENGTH];

uint8_t DevicesRequestBuffer[DEVICES_REQUEST_LENGTH];
uint8_t DevicesResponseBuffer[DEVICES_NUMBER][DEVICES_RESPONSE_LENGTH];

uint8_t PressureResponseBuffer[PRESSURE_SENSOR_SIZE];
