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
	rThrusters[FrLowR	].address = 0;
	rThrusters[FrLowL	].address = 1;
	rThrusters[BackLowR	].address = 2;
	rThrusters[BackLowL	].address = 3;
	rThrusters[FrUpR	].address = 4;//xer
	rThrusters[FrUpL	].address = 5;
	rThrusters[BackUpL	].address = 6;
	rThrusters[BackUpR	].address = 7;


	rThrusters[FrLowR	].inverse = false;
	rThrusters[FrLowL	].inverse = false;
	rThrusters[BackLowR	].inverse = false;
	rThrusters[BackLowL	].inverse = false;
	rThrusters[FrUpR	].inverse = false;
	rThrusters[FrUpL	].inverse = false;
	rThrusters[BackUpL	].inverse = false;
	rThrusters[BackUpR	].inverse = false;

	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		rThrusters[i].desiredSpeed = 0;
		rThrusters[i].kForward = 0.5;
		rThrusters[i].kBackward = 0.5;
		rThrusters[i].sForward = 127;
		rThrusters[i].sBackward = 127;
	}
}

void resetThrusters()
{
	rJoySpeed.depth = 0;
	rJoySpeed.lag = 0;
	rJoySpeed.march = 0;
	rJoySpeed.pitch = 0;
	rJoySpeed.roll = 0;
	rJoySpeed.yaw = 0;

	rThrusters[FrLowR	].desiredSpeed = 0;
	rThrusters[FrLowL	].desiredSpeed = 0;
	rThrusters[BackLowR	].desiredSpeed = 0;
	rThrusters[BackLowL	].desiredSpeed = 0;
	rThrusters[FrUpR	].desiredSpeed = 0;
	rThrusters[FrUpL	].desiredSpeed = 0;
	rThrusters[BackUpL	].desiredSpeed = 0;
	rThrusters[BackUpR	].desiredSpeed = 0;
}

void fillThrustersRequest(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = rThrusters[thruster].address;
    int16_t velocity = rThrusters[thruster].desiredSpeed;

    // Inverting
    if(rThrusters[thruster].inverse) {
    	velocity *= -1;
    }

    // Multiplier constants
    if(velocity > 0) {
    	velocity = (int16_t) ((float) (velocity) * rThrusters[thruster].kForward);
    }
    else if(velocity < 0) {
    	velocity = (int16_t) ((float) (velocity) * rThrusters[thruster].kBackward);
    }

    // Saturation
    if(velocity > rThrusters[thruster].sForward) {
    	velocity = rThrusters[thruster].sForward;
    }
    else if(velocity < -rThrusters[thruster].sBackward) {
    	velocity = -rThrusters[thruster].sBackward;
    }
    res.velocity = velocity;

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
	float k10 = -0.2041149;
	float k11 = 0.25000000;
	float k12 = 0.35350677;
	float k13 = 1.00000000;
	float k14 = -0.0061806;
	float k15 = -1.0000000;
	float k16 = 1.00000000;
	float k20 = -0.2041149;
	float k21 = -0.2500000;
	float k22 = -0.3535067;
	float k23 = -1.0000000;
	float k24 = -0.0061806;
	float k25 = 1.00000000;
	float k26 = 1.00000000;
	float k30 = -0.2041149;
	float k31 = 0.25000000;
	float k32 = -0.3535067;
	float k33 = -1.0000000;
	float k34 = -0.0061806;
	float k35 = -1.0000000;
	float k36 = 1.00000000;
	float k40 = -0.2041149;
	float k41 = -0.2500000;
	float k42 = 0.35350677;
	float k43 = 1.00000000;
	float k44 = -0.0061806;
	float k45 = 1.00000000;
	float k46 = 1.00000000;
	float k50 = 0.20411496;
	float k51 = -0.2500000;
	float k52 = 0.35350677;
	float k53 = 1.00000000;
	float k54 = -0.0061806;
	float k55 = 1.00000000;
	float k56 = 1.00000000;
	float k60 = 0.20411496;
	float k61 = 0.25000000;
	float k62 = -0.3535067;
	float k63 = -1.0000000;
	float k64 = -0.0061806;
	float k65 = -1.0000000;
	float k66 = 1.00000000;
	float k70 = -0.2041149;
	float k71 = 0.25000000;
	float k72 = 0.35350677;
	float k73 = 1.00000000;
	float k74 = 0.00618065;
	float k75 = -1.0000000;
	float k76 = 1.00000000;
	float k80 = 0.20411496;
	float k81 = 0.25000000;
	float k82 = -0.3535067;
	float k83 = -1.0000000;
	float k84 = -0.0061806;
	float k85 = 1.00000000;
	float k86 = 1.00000000;

	float velocity[THRUSTERS_NUMBER];
	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
		velocity[i] = 0;
	}
	float Ux;
	float Uy;
	float Uz;
	float Ugamma;
	float Uteta;
	float Upsi;

	rStabConstants[STAB_MARCH	].enable ?  Ux = rStabState[STAB_MARCH	].outputSignal : Ux = rJoySpeed.march;
	rStabConstants[STAB_LAG		].enable ?  Uy = rStabState[STAB_LAG	].outputSignal : Uy = rJoySpeed.lag;
	rStabConstants[STAB_DEPTH	].enable ?  Uz = rStabState[STAB_DEPTH	].outputSignal : Uz = rJoySpeed.depth;
	rStabConstants[STAB_YAW		].enable ?  Upsi = rStabState[STAB_YAW	].outputSignal : Upsi = rJoySpeed.yaw;
	rStabConstants[STAB_ROLL	].enable ?  Ugamma = rStabState[STAB_ROLL	].outputSignal : Ugamma = rJoySpeed.roll;
	rStabConstants[STAB_PITCH	].enable ?  Uteta = rStabState[STAB_PITCH	].outputSignal : Uteta = rJoySpeed.pitch;

	velocity[FrLowR		] = (k10*Ux + k11*Uy + k12*Uz + k13*Ugamma + k14*Uteta + k15*Upsi)*k16;
	velocity[FrLowL		] = (k20*Ux + k21*Uy + k22*Uz + k23*Ugamma + k24*Uteta + k25*Upsi)*k26;
	velocity[BackLowR	] = (k30*Ux + k31*Uy + k32*Uz + k33*Ugamma + k34*Uteta + k35*Upsi)*k36;
	velocity[BackLowL	] = (k40*Ux + k41*Uy + k42*Uz + k43*Ugamma + k44*Uteta + k45*Upsi)*k46;
	velocity[FrUpR		] = (k50*Ux + k51*Uy + k52*Uz + k53*Ugamma + k54*Uteta + k55*Upsi)*k56;
	velocity[FrUpL		] = (k60*Ux + k61*Uy + k62*Uz + k63*Ugamma + k64*Uteta + k65*Upsi)*k66;
	velocity[BackUpL	] = (k70*Ux + k71*Uy + k72*Uz + k73*Ugamma + k74*Uteta + k75*Upsi)*k76;
	velocity[BackUpR	] = (k80*Ux + k81*Uy + k82*Uz + k83*Ugamma + k84*Uteta + k85*Upsi)*k86;

	for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i) {
		rThrusters[i].desiredSpeed = resizeFloatToUint8(velocity[i]);
	}
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
