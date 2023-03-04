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

uint8_t ImuRequestBuffer[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAE, 0x01, 0xFF };
uint8_t ImuResetRequestBuffer[IMU_REQUEST_LENGTH] = { 's', 'n', 'p', 0x00, 0xAC, 0x01, 0xFD };
uint8_t ImuResponseBuffer[IMU_RESPONSE_LENGTH*IMU_CHECKSUMS];

uint8_t ThrustersRequestBuffer[THRUSTERS_REQUEST_LENGTH];
uint8_t ThrustersResponseBuffer[THRUSTERS_NUMBER][THRUSTERS_RESPONSE_LENGTH];

uint8_t DevicesRequestBuffer[DEVICES_REQUEST_LENGTH];
uint8_t DevicesResponseBuffer[DEVICES_NUMBER][DEVICES_RESPONSE_LENGTH];

uint8_t PressureResponseBuffer[PRESSURE_SENSOR_SIZE];
