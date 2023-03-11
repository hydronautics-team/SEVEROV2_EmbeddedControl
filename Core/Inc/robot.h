#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#include "messages.h"

enum VMA {
	FrLowR = 0,
    FrLowL,
    BackLowR,
    BackLowL,
    FrUpR,
	FrUpL,
    BackUpL,
	BackUpR
};

#define DEV_AMOUNT 6

enum DEV {
    GRAB = 0,
    GRAB_ROTATION,
    TILT,
    DEV1,
	DEV2,
    DEV3
};

#define LOGDEV_AMOUNT 1

enum LOGDEV {
	LOGDEV_LIFTER = 0
};

#define STABILIZATION_AMOUNT 6

enum STAB_CIRCUITS {
	STAB_DEPTH = 0,
	STAB_MARCH,
	STAB_LAG,
	STAB_YAW,
	STAB_ROLL,
	STAB_PITCH
};

#define STABILIZATION_FILTERS 3

enum STAB_FILTERS {
	POS_FILTER = 0,
	SPEED_FILTER,
	THRUSTERS_FILTER
};

enum I2C {
	I2C_SENSORS = 0,
	I2C_PC
};

enum OPERATION_MODES {
	OP_NORMAL = 0,
	OP_QUALIFICATION
};

enum LOGDEV_STATE {
	LOGDEV_NULL = 0,
	LOGDEV_FORWARD,
	LOGDEV_BACKWARD,
	LOGDEV_FORWARD_SAT,
	LOGDEV_BACKWARD_SAT
};

#define PC_POWERON_DELAY 2

// Structure for overall robot state
struct robotState_s {
	uint8_t cameraNum;	// Current camera ID (controls multiplexor)
	uint8_t contourSelected; // Current contour selected for the configuration mode
	uint8_t flash; // Was flash read successful
	uint8_t operationMode; // Currrent operation type
	uint8_t pcCounter;
	uint8_t thrustersOn
	float lag_error;
};

// Individual characteristics, controls and current state of each VMA
struct robotThrusters_s {
	// current controls (data ready to fetch)
	uint8_t address;
	uint8_t settings;
	int8_t desiredSpeed;
	// current state (fresh data)
	uint16_t current;
	uint8_t errors;
	int8_t realSpeed;
	int8_t speedError;
	// characteristics (this parameters affects only sending data, they are not meant to be sent or received)
	float kForward;		// this constants will be multiplied with desiredSpeed
	float kBackward;
	int8_t sForward; 	// thresholds for thruster signal
	int8_t sBackward;
	bool inverse; 		// inverts thruster
};

struct robotSensors_s {
	float roll;
	float pitch;
	float yaw;
	float raw_yaw;

	float old_yaw;
	float LastTick;
	int16_t spins;

	float rollSpeed;
	float pitchSpeed;
	float yawSpeed;
	float accelX;
	float accelY;
	float accelZ;
	float magX;
	float magY;
	float magZ;
	float quatA;
	float quatB;
	float quatC;
	float quatD;
	bool resetIMU;

	float pressure;
	float pressure_null;
	float velocity_pressure;

	float in_pressure;

	float leak;
};

struct robotPc_s {
	uint8_t reset;
	uint8_t errors;
};

struct robotJoystickSpeed_s {
	float march;
	float lag;
	float depth;
	float roll;
	float pitch;
	float yaw;
};

struct robotPositionMovement_s {
	float march;
	float lag;
	float depth;
	float roll;
	float pitch;
	float yaw;
};

struct robotDevices_s {
	uint8_t address;
	uint8_t settings;
	int8_t force;
	uint16_t current;
	uint8_t errors;

	uint8_t velocity1;
	uint8_t velocity2;
};

struct RobotLogicDevices_s {
	uint8_t state;
	uint8_t control;
	uint8_t errors;
};

struct robotStabilizationConstants_s {
	bool enable;
	bool joyIntegration; // allow integration of joy casted value
	// Before P
	float pJoyUnitCast;
	float pSpeedDyn;
	float pErrGain;
	// Feedback aperiodic filters
	struct AperiodicFilter {
		float T;
		float K;
	} aFilter[STABILIZATION_FILTERS];
	// PID
	struct pidRegulator {
		float pGain;
		float iGain;
		float iMax;
		float iMin;
	} pid;
	// Thrusters unit cast
	float pThrustersMin;
	float pThrustersMax;
	// Output summator saturation
	float sOutSummatorMax;
	float sOutSummatorMin;
};

struct robotStabilizationState_s {
	float *inputSignal; 		// Link to input signal. You need to set this on initialization
	float *speedSignal;			// Link to speed signal. You need to set this on initialization
	float *posSignal;			// Link to position signal. You need to set this on initialization

	float speedIntegral;		// Integral from speed (if you don't have separate position signal)
	float posDerivative;		// Derivative from position (if you don't have separate speed signal)

	float oldSpeed;
	float oldPos;

	float joyUnitCasted;
	float joy_iValue;
	float posError;
	float speedError;
	float dynSummator;
	float pidValue;
	float pid_iValue;
	float posErrorAmp;
	float speedFiltered;
	float posFiltered;
	float oldPosFiltered;
	float oldSpeedError;
	float thrustersFiltered;

	float outputSignal;

	float LastTick;
};


#endif
