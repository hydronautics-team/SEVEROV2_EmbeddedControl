#include "stabilization.h"

#include "FreeRTOSTick.h"
#include "math.h"
#include "robot.h"

void stabilizationInit()
{
	for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
		rStabConstants[i].enable = false;

		rStabState[i].speedIntegral = 0;
		rStabState[i].posDerivative = 0;
		rStabState[i].oldSpeed = 0;
		rStabState[i].oldPos = 0;

		rStabState[i].joyUnitCasted = 0;
		rStabState[i].joy_iValue = 0;
		rStabState[i].posError = 0;
		rStabState[i].speedError = 0;
		rStabState[i].dynSummator = 0;
		rStabState[i].pidValue = 0;
		rStabState[i].pid_iValue = 0;
		rStabState[i].posErrorAmp = 0;
		rStabState[i].speedFiltered = 0;
		rStabState[i].posFiltered = 0;
		rStabState[i].oldPosFiltered = 0;
		rStabState[i].oldSpeedError = 0;
		rStabState[i].thrustersFiltered = 0;
		rStabState[i].outputSignal = 0;

		rStabState[i].LastTick = 0;

		if(!rState.flash) {
			rStabConstants[i].pJoyUnitCast = 1;
			rStabConstants[i].pSpeedDyn = 1;
			rStabConstants[i].pErrGain = 1;
			rStabConstants[i].aFilter[SPEED_FILTER].T = 0;
			rStabConstants[i].aFilter[SPEED_FILTER].K = 1;
			rStabConstants[i].aFilter[POS_FILTER].T = 0;
			rStabConstants[i].aFilter[POS_FILTER].K = 1;
			rStabConstants[i].pid.pGain = 1;
			rStabConstants[i].pid.iGain = 1;
			rStabConstants[i].pid.iMax = -1000;
			rStabConstants[i].pid.iMin = 1000;
			rStabConstants[i].pThrustersMax = 5000;
			rStabConstants[i].pThrustersMin = -5000;
			rStabConstants[i].sOutSummatorMax = 32000;
			rStabConstants[i].sOutSummatorMin = -32000;
		}
	}




	/////////////////////////////////////////////////////////////
    rStabState[STAB_ROLL].inputSignal = &rJoySpeed.roll;
    rStabState[STAB_ROLL].speedSignal = &rSensors.rollSpeed;
    rStabState[STAB_ROLL].posSignal = &rSensors.roll;
    rStabConstants[STAB_ROLL].joyIntegration = false;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_PITCH].inputSignal = &rJoySpeed.pitch;
    rStabState[STAB_PITCH].speedSignal = &rSensors.pitchSpeed;
    rStabState[STAB_PITCH].posSignal = &rSensors.pitch;
    rStabConstants[STAB_PITCH].joyIntegration = true;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_YAW].inputSignal = &rJoySpeed.yaw;
    rStabState[STAB_YAW].speedSignal = &rSensors.yawSpeed;
    rStabState[STAB_YAW].posSignal = &rSensors.yaw;//&rStabState[STAB_YAW].speedIntegral;
    rStabConstants[STAB_YAW].joyIntegration = true;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_DEPTH].inputSignal = &rJoySpeed.depth;
    rStabState[STAB_DEPTH].speedSignal = &rSensors.velocity_pressure;//&rStabState[STAB_DEPTH].posDerivative;
    rStabState[STAB_DEPTH].posSignal = &rSensors.pressure;
    rStabConstants[STAB_DEPTH].joyIntegration = false;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_LAG].inputSignal = &rJoySpeed.lag;
    rStabState[STAB_LAG].speedSignal = &rStabState[STAB_LAG].posDerivative;
    rStabState[STAB_LAG].posSignal = &rState.lag_error;
    rStabConstants[STAB_LAG].joyIntegration = false;
    /////////////////////////////////////////////////////////////
    rStabState[STAB_MARCH].inputSignal = &rJoySpeed.march;
    rStabState[STAB_MARCH].speedSignal = &rStabState[STAB_MARCH].posDerivative;
    rStabState[STAB_MARCH].posSignal = &rJoySpeed.march;
    rStabConstants[STAB_MARCH].joyIntegration = false;


    //upload coef
	//rStabConstants[STAB_YAW].enable = true;

	rStabConstants[STAB_YAW].pJoyUnitCast = 1;
	rStabConstants[STAB_YAW].pSpeedDyn = 0;
	rStabConstants[STAB_YAW].pErrGain = 70;
	rStabConstants[STAB_YAW].aFilter[SPEED_FILTER].T = 0;
	rStabConstants[STAB_YAW].aFilter[SPEED_FILTER].K = 120;
	rStabConstants[STAB_YAW].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_YAW].aFilter[POS_FILTER].K = 1;
	rStabConstants[STAB_YAW].pid.pGain = 25;
	rStabConstants[STAB_YAW].pid.iGain = 0;
	rStabConstants[STAB_YAW].pid.iMax = 3000;
	rStabConstants[STAB_YAW].pid.iMin = -3000;
	rStabConstants[STAB_YAW].pThrustersMax = 20000;
	rStabConstants[STAB_YAW].pThrustersMin = -20000;
	rStabConstants[STAB_YAW].sOutSummatorMax = 32000;
	rStabConstants[STAB_YAW].sOutSummatorMin = -32000;
    rStabConstants[STAB_YAW].joyIntegration = false;

	//rStabConstants[STAB_DEPTH].enable = true;

	rStabConstants[STAB_DEPTH].pJoyUnitCast = -1;
	rStabConstants[STAB_DEPTH].pSpeedDyn = 1;
	rStabConstants[STAB_DEPTH].pErrGain = 1;
	rStabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].T = 80;
	rStabConstants[STAB_DEPTH].aFilter[SPEED_FILTER].K = -15;
	rStabConstants[STAB_DEPTH].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_DEPTH].aFilter[POS_FILTER].K = -1;
	rStabConstants[STAB_DEPTH].pid.pGain = 30;
	rStabConstants[STAB_DEPTH].pid.iGain = 5;
	rStabConstants[STAB_DEPTH].pid.iMax = 40;
	rStabConstants[STAB_DEPTH].pid.iMin = -40;
	rStabConstants[STAB_DEPTH].pThrustersMax = 32000;
	rStabConstants[STAB_DEPTH].pThrustersMin = -32000;
	rStabConstants[STAB_DEPTH].sOutSummatorMax = 32000;
	rStabConstants[STAB_DEPTH].sOutSummatorMin = -32000;
    rStabConstants[STAB_DEPTH].joyIntegration = false;

	rStabConstants[STAB_PITCH].pJoyUnitCast = -1;
	rStabConstants[STAB_PITCH].pSpeedDyn = 0;
	rStabConstants[STAB_PITCH].pErrGain = 1;
	rStabConstants[STAB_PITCH].aFilter[SPEED_FILTER].T = 0;
	rStabConstants[STAB_PITCH].aFilter[SPEED_FILTER].K = 1;
	rStabConstants[STAB_PITCH].aFilter[POS_FILTER].T = 0;
	rStabConstants[STAB_PITCH].aFilter[POS_FILTER].K = -100;
	rStabConstants[STAB_PITCH].pid.pGain = 25;
	rStabConstants[STAB_PITCH].pid.iGain = 60;
	rStabConstants[STAB_PITCH].pid.iMax = 50;
	rStabConstants[STAB_PITCH].pid.iMin = -50;
	rStabConstants[STAB_PITCH].pThrustersMax = 32000;
	rStabConstants[STAB_PITCH].pThrustersMin = -32000;
	rStabConstants[STAB_PITCH].sOutSummatorMax = 32000;
	rStabConstants[STAB_PITCH].sOutSummatorMin = -32000;
    rStabConstants[STAB_PITCH].joyIntegration = false;


}

void stabilizationStart(uint8_t contour)
{
	rStabConstants[contour].enable = true;

	rStabState[contour].oldSpeed = *rStabState[contour].speedSignal;
	rStabState[contour].oldPos = *rStabState[contour].posSignal;
	rStabState[contour].posDerivative = 0;
	//rStabState[contour].speedIntegral = 0;
if(contour==STAB_YAW)
{
	rSensors.yaw =0;
}
	rStabState[contour].joyUnitCasted = 0;
	rStabState[contour].joy_iValue = *rStabState[contour].posSignal;
	rStabState[contour].posError = 0;
	rStabState[contour].speedError = 0;
	rStabState[contour].dynSummator = 0;
	rStabState[contour].pidValue = 0;
	rStabState[contour].pid_iValue = 0;
	rStabState[contour].posErrorAmp = 0;
	rStabState[contour].speedFiltered = 0;
	rStabState[contour].posFiltered = 0;
	rStabState[contour].oldPosFiltered = 0;
	rStabState[contour].oldSpeedError = 0;
	rStabState[contour].thrustersFiltered = 0;
	rStabState[contour].outputSignal = 0;
	rStabState[contour].LastTick = xTaskGetTickCount();
}

void stabilizationUpdate(uint8_t contour)
{
	struct robotStabilizationConstants_s *constants = &rStabConstants[contour];
	struct robotStabilizationState_s *state = &rStabState[contour];
	float diffTime = fromTickToMs(xTaskGetTickCount() - state->LastTick) / 1000.0f;
	state->LastTick = xTaskGetTickCount();

	// Speed feedback filtering
	struct AperiodicFilter *filter = &constants->aFilter[SPEED_FILTER];

	if(filter->T != 0) {
		//state->speedFiltered = state->speedFiltered*exp(-diffTime/filter->T/1e5) + state->oldSpeed*filter->K*80*(1-exp(-diffTime/filter->T/1e5));
		state->speedFiltered = state->oldSpeed + diffTime * (1/filter->T*100) * (*state->speedSignal * filter->K*70 - state->oldSpeed);
	}
	else {
		if(contour==STAB_PITCH)
		{
			state->speedFiltered = *state->speedSignal*filter->K*250;
		}
		else
		state->speedFiltered = *state->speedSignal*filter->K*10;
	}
	//state->oldSpeed = *state->speedSignal;
	state->oldSpeed = state->speedFiltered;

	// Position feedback filtering
	filter = &constants->aFilter[POS_FILTER];
	if(filter->T != 0) {
		state->posFiltered = state->posFiltered*exp(-diffTime/filter->T) + state->oldPos*filter->K*(1-exp(-diffTime/filter->T));
	}
	else {
		state->posFiltered = *state->posSignal*filter->K;
	}
	state->oldPos = *state->posSignal;

	// Speed integration calculation
	state->speedIntegral += (*state->speedSignal * diffTime);

    // Position derivative calculation
    state->posDerivative = (state->posFiltered - state->oldPosFiltered) / diffTime;
    state->oldPosFiltered = state->posFiltered;

	// Input signal unit cast
	state->joyUnitCasted = constants->pJoyUnitCast * *state->inputSignal;

    // Casted input signal integration
	if(constants->joyIntegration) {
		state->joy_iValue += state->joyUnitCasted * diffTime;
	}
	else {
		state->joy_iValue = state->joyUnitCasted;
	}

    // Position feedback summator
    state->posError = state->joy_iValue - state->posFiltered;

    // Feedback amplifiers
    state->posErrorAmp = state->posError * constants->pErrGain;

    // PI integration
    state->pid_iValue += (state->posErrorAmp * diffTime) * constants->pid.iGain;

    // PI integration saturation
    if(state->pid_iValue > constants->pid.iMax*100) {
    	state->pid_iValue = constants->pid.iMax*100;
    }
    else if(state->pid_iValue < constants->pid.iMin*100) {
    	state->pid_iValue = constants->pid.iMin*100;
    }

    // PI summator
    state->pidValue =  state->pid_iValue + (state->posErrorAmp * constants->pid.pGain);

    // Dynamic summator
    state->dynSummator = state->pidValue + *state->inputSignal * constants->pSpeedDyn;

    // Speed feedback
    state->speedError = state->dynSummator - state->speedFiltered;

    // Out filtering
    filter = &constants->aFilter[THRUSTERS_FILTER];
    if(filter->T != 0) {
    	state->thrustersFiltered = state->thrustersFiltered*exp(-diffTime/filter->T) + state->oldSpeedError*filter->K*(1-exp(-diffTime/filter->T));
    }
    else {
    	state->thrustersFiltered = state->speedError*filter->K;
    }
    state->oldSpeedError = state->speedError;

    if(state->thrustersFiltered > constants->pThrustersMax) {
    	state->thrustersFiltered = constants->pThrustersMax;
    }
    else if(state->thrustersFiltered < constants->pThrustersMin) {
    	state->thrustersFiltered = constants->pThrustersMin;
    }

    state->outputSignal = -1*state->speedError;
}

