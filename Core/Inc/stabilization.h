#ifndef STABILIZATION_H
#define STABILIZATION_H

#include <math.h>
#include <stdbool.h>

#include "FreeRTOSTick.h"
#include "robot.h"
#include "global.h"

struct PidRegulator_s{
	TickType_t lastUpdateTick;
	float iGain;
	float dGain;
	float pGain;
	float dState;
	float iState;
	float iMax;
	float iMin;

	float dTermLast;
	float pTermLast;
	float iTermLast;
};

void stabilizationInit();
void stabilizationStart(uint8_t contour);
void stabilizationUpdate(uint8_t contour);

void pidInit(struct PidRegulator_s *pid, float pGain, float iGain, float iMax, float iMin, float dGain);
float pidUpdate(struct PidRegulator_s *pid, float error, float deltaTime);
void pidReset(struct PidRegulator_s *pid);
void updatePidConstants();

#endif
