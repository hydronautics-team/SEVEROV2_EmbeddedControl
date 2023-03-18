#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "robot.h"
#include "messages.h"
#include "global.h"
#include "communication.h"
#include "checksum.h"

void addMarchToSumm(float *velocity);
void addLagToSumm(float *velocity);
void addDepthToSumm(float *velocity);
void addYawToSumm(float *velocity);
void addRollToSumm(float *velocity);
void addPitchToSumm(float *velocity);

uint8_t resizeFloatToUint8(float input);

void thrustersInit()
{
	rThrusters[Lag1st].address = 1;
	rThrusters[Lag2nd].address = 7;

	rThrusters[MarshLEFT].address = 3;
	rThrusters[MarshRIGHT].address = 2;

	rThrusters[MarshDown].address = 5;//xer

	rThrusters[VertBACK].address 	= 6;

	rThrusters[VertLEFT].address 	= 8;
	rThrusters[VertRIGHT].address 	= 4;


	rThrusters[Lag1st].inverse = true;
	rThrusters[Lag2nd].inverse = true;
	rThrusters[MarshLEFT].inverse = true;
	rThrusters[MarshRIGHT].inverse = true;
	rThrusters[MarshDown].inverse = true;

	rThrusters[VertBACK].inverse 	= false;
	rThrusters[VertLEFT].inverse 	= false;
	rThrusters[VertRIGHT].inverse 	= true;

	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		rThrusters[i].desiredSpeed = 0;
		rThrusters[i].kForward = 0.4;
		rThrusters[i].kBackward = 0.4;
		rThrusters[i].sForward = 127;
		rThrusters[i].sBackward = 127;
	}

	rThrusters[VertBACK].kForward = 0.4;
	rThrusters[VertBACK].kBackward = 0.4;
	rThrusters[VertLEFT].kForward = 0.7;
	rThrusters[VertLEFT].kBackward = 0.7;
	rThrusters[VertRIGHT].kForward = 0.7;
	rThrusters[VertRIGHT].kBackward = 0.7;

	rThrusters[MarshLEFT].kForward = 0.9;
	rThrusters[MarshLEFT].kBackward = 0.9;
	rThrusters[MarshRIGHT].kForward = 0.9;
	rThrusters[MarshRIGHT].kBackward = 0.9;

	rThrusters[MarshDown].kForward = 0.7;
	rThrusters[MarshDown].kBackward = 0.7;
}

void resetThrusters()
{
	rJoySpeed.depth = 0;
	rJoySpeed.lag = 0;
	rJoySpeed.march = 0;
	rJoySpeed.pitch = 0;
	rJoySpeed.roll = 0;
	rJoySpeed.yaw = 0;

	rThrusters[Lag1st].desiredSpeed = 0;
	rThrusters[Lag2nd].desiredSpeed = 0;
	rThrusters[MarshLEFT].desiredSpeed = 0;
	rThrusters[MarshRIGHT].desiredSpeed = 0;
	rThrusters[VertBACK].desiredSpeed = 0;
	rThrusters[VertLEFT].desiredSpeed = 0;
	rThrusters[VertRIGHT].desiredSpeed = 0;
	rThrusters[MarshDown].desiredSpeed = 0;
}

void fillThrustersRequest(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address =0xAF;
    for(int i =0;i<8;i++){


    int16_t velocity = rThrusters[i].desiredSpeed;

    // Inverting
    if(rThrusters[i].inverse) {
    	velocity *= -1;
    }

    // Multiplier constants
    if(velocity > 0) {
    	velocity = (int16_t) ( (float) (velocity) * rThrusters[i].kForward);
    }
    else if(velocity < 0) {
    	velocity = (int16_t) ((float) (velocity) * rThrusters[i].kBackward);
    }

    // Saturation
//    if(velocity > rThrusters[thruster].sForward) {
//    	velocity = rThrusters[thruster].sForward;
//    }
//    else if(velocity < -rThrusters[thruster].sBackward) {
//    	velocity = -rThrusters[thruster].sBackward;
//    }

    res.velocity[i] = velocity;
    }

    memcpy((void*)buf, (void*)&res, THRUSTERS_REQUEST_LENGTH);

    AddChecksumm8bVma(buf, THRUSTERS_REQUEST_LENGTH);
}

void fillThrustersResponse(uint8_t *buf, uint8_t thruster)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, THRUSTERS_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct thrustersResponse_s res;
    	memcpy((void*)&res, (void*)buf, THRUSTERS_RESPONSE_LENGTH);

        rThrusters[thruster].current = res.current;

        ++uartBus[THRUSTERS_UART].successRxCounter;
    }
    else {
    	++uartBus[THRUSTERS_UART].brokenRxCounter;
    }
}

void formThrustVectors()
{
	float velocity[THRUSTERS_NUMBER];
	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		velocity[i] = 0;
	}
	// March thrusters1
	addMarchToSumm(velocity);
	// Lag Thrusters
	addYawToSumm(velocity);
	addLagToSumm(velocity);
	// Two vertical thrusters
	addDepthToSumm(velocity);
	addRollToSumm(velocity);
	// One vertical corrective thruster
	addPitchToSumm(velocity);

	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i) {
		rThrusters[i].desiredSpeed = resizeFloatToUint8(velocity[i]);
	}
}

void addMarchToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_MARCH].enable) {
		value = rStabState[STAB_MARCH].outputSignal;
	}
	else {
		value = rJoySpeed.march;
	}
	// March contour summ
	velocity[MarshLEFT] += value;
	velocity[MarshRIGHT] += value;
	velocity[MarshDown] += value;

	// March summ saturation
//	for(uint8_t i=MarshLEFT; i<Lag2nd+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_MARCH].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_MARCH].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_MARCH].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_MARCH].sOutSummatorMin;
//		}
//	}
}

void addLagToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_LAG].enable) {
		value = rStabState[STAB_LAG].outputSignal;
	}
	else {
		value = rJoySpeed.lag;
	}
	// Lag contour summ
	velocity[Lag1st] += value;
	velocity[Lag2nd] += value;
	// Lag summ saturation
//	for(uint8_t i=MarshLEFT; i<Lag2nd+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_LAG].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_LAG].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_LAG].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_LAG].sOutSummatorMin;
//		}
//	}
}

void addDepthToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_DEPTH].enable) {
		value = rStabState[STAB_DEPTH].outputSignal;
	}
	else {
		value = rJoySpeed.depth;
	}
	// Depth contour summ
	//velocity[VertBACK] -= value;
	velocity[VertLEFT] -= value;
	velocity[VertRIGHT] -= value;
	// Depth summ saturation
//	for(uint8_t i=VertBACK; i<VertRIGHT+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_DEPTH].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_DEPTH].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_DEPTH].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_DEPTH].sOutSummatorMin;
//		}
//	}
}

void addYawToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_YAW].enable) {
		value = rStabState[STAB_YAW].outputSignal;
	}
	else {
		value = rJoySpeed.yaw;
	}
	// Yaw contour summ
//	velocity[MarshLEFT] += value;
//	velocity[MarshRIGHT] -= value;
	velocity[Lag1st] += value;
	velocity[Lag2nd] -= value;
	// Yaw summ saturation
//	for(uint8_t i=MarshLEFT; i<Lag2nd+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_YAW].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_YAW].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_YAW].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_YAW].sOutSummatorMin;
//		}
//	}
}

void addRollToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_ROLL].enable) {
		value = rStabState[STAB_ROLL].outputSignal;
	}
	else {
		value = rJoySpeed.roll;
	}
	// Yaw contour summ
	velocity[VertBACK] += value;
	//velocity[MarshDown] += value;

	// Yaw summ saturation
//	for(uint8_t i=VertBACK; i<VertRIGHT+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_ROLL].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_ROLL].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_ROLL].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_ROLL].sOutSummatorMin;
//		}
//	}
}

void addPitchToSumm(float *velocity)
{
	float value = 0;
	// Choosing source of the signal
	if(rStabConstants[STAB_PITCH].enable) {
		value = rStabState[STAB_PITCH].outputSignal;
	}
	else {
		value = rJoySpeed.pitch;
	}
	velocity[VertBACK] += value;

	velocity[VertLEFT] -= 0.4*value;
	velocity[VertRIGHT] -= 0.4*value;

	// Pitch contour summ
//	velocity[VertLEFT] += value;
//	velocity[VertRIGHT] -= value;
	// Pitch summ saturation
//	for(uint8_t i=VertLEFT; i<VertRIGHT+1; i++) {
//		if(velocity[i] > rStabConstants[STAB_PITCH].sOutSummatorMax) {
//			velocity[i] = rStabConstants[STAB_PITCH].sOutSummatorMax;
//		}
//		else if(velocity[i] < rStabConstants[STAB_PITCH].sOutSummatorMin) {
//			velocity[i] = rStabConstants[STAB_PITCH].sOutSummatorMin;
//		}
//	}
}

uint8_t resizeFloatToUint8(float input)
{
	int32_t cast = (int32_t) input;
	cast = cast / 0xFF;
	if (cast > 127) {
		cast = 127;
	}
	else if(cast < -127) {
		cast = -127;
	}
	return (int8_t) cast;
}
