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


float KVMA[8][7] = {
    {-0.2041149, -0.25000000, 0.35350677, 1.00000000, -0.0061806, 1.0000000, -1.00000000},
    {-0.2041149, 0.25000000, 0.3535067, -1.0000000, -0.0061806,  -1.00000000, 1.00000000},
    {0.2041149, -0.25000000, 0.3535067, -1.0000000, -0.0061806, -1.0000000, 1.00000000},
    {0.2041149, 0.2500000, 0.35350677, 1.00000000, -0.0061806, 1.00000000, 1.00000000},
    {-0.20411496, -0.2500000, -0.35350677, 1.00000000, -0.0061806, 1.00000000, 1.00000000},
    {-0.20411496, 0.25000000, -0.3535067, -1.0000000, -0.0061806, -1.0000000, -1.00000000},
    {0.2041149, 0.25000000, -0.35350677, 1.00000000, 0.00618065, 1.0000000, 1.00000000},
    {0.20411496, -0.25000000, -0.3535067, -1.0000000, -0.0061806, -1.00000000,  1.00000000}
};

void thrustersInit()
{
   //Numarate by LOVE
  rThrusters[FDR].address = 1; //Forward Down Right
  rThrusters[FDL].address = 2; //Forward Down Left
  rThrusters[BDR].address = 3; //Back Down Right
  rThrusters[BDL].address = 4; //Back Down Left

  rThrusters[FUR].address = 5; //Forward Up Right
  rThrusters[FUL].address = 6; //Forward Up Left
  rThrusters[BUR].address = 7; //Back Up Right
  rThrusters[BUL].address = 8; //Back Up Left

  for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
    rThrusters[i].desiredSpeed = 0;
    rThrusters[i].kForward = 1;
    rThrusters[i].kBackward =1;
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

  for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
    rThrusters[i].desiredSpeed = 0;
  }

}

void fillThrustersRequest(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = 0xAF;
    for(int i=0; i<8;i++){
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

  float Ux;
  float Uy;
  float Uz;
  float Ugamma;
  float Uteta;
  float Upsi;

  Ux = rStabConstants[STAB_MARCH].enable ?  rStabState[STAB_MARCH].outputSignal : rJoySpeed.march;
  Uy = rStabConstants[STAB_LAG].enable ?   rStabState[STAB_LAG].outputSignal : rJoySpeed.lag;
  Uz = rStabConstants[STAB_DEPTH].enable ?  rStabState[STAB_DEPTH].outputSignal : rJoySpeed.depth;
  Upsi =  rStabConstants[STAB_YAW].enable ?   rStabState[STAB_YAW].outputSignal :  rJoySpeed.yaw;
  Ugamma =rStabConstants[STAB_ROLL].enable ?   rStabState[STAB_ROLL].outputSignal : rJoySpeed.roll;
  Uteta = rStabConstants[STAB_PITCH].enable ?   rStabState[STAB_PITCH].outputSignal :  rJoySpeed.pitch;

  for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i)
  {
    velocity[i] = (KVMA[i][0]*Ux + KVMA[i][1]*Uy + KVMA[i][2]*Uz
        + KVMA[i][3]*Ugamma + KVMA[i][4]*Uteta + KVMA[i][5]*Upsi)*KVMA[i][6];
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
