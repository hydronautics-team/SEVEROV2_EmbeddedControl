#ifndef FLASH_H
#define FLASH_H

#include "stdint.h"
#include "stdbool.h"
#include <string.h>
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "communication.h"

#pragma pack(push, 1)

struct flashConfiguration_s {
	uint8_t writeFlag;
	struct flashStabConstants_s {
		// Before P
		float pJoyUnitCast;
		float pSpeedDyn;
		float pErrGain;
		// Feedback aperiodic filters
		float aFilter_T1;
		float aFilter_T2;
		float aFilter_K1;
		float aFilter_K2;
		// PID
		float pid_pGain;
		float pid_iGain;
		float pid_iMax;
		float pid_iMin;
		// Thrusters unit cast
		float pThrustersMin;
		float pThrustersMax;
		// Output aperiodic filter
		float aFilter_thrusters_T;
		float aFilter_thrusters_K;
		// Output summator saturation
		float sOutSummatorMax;
		float sOutSummatorMin;
	} stabConstants[STABILIZATION_AMOUNT];

	struct flashThrusters_s {
		uint8_t address;
		float kForward;
		float kBackward;
		float sForward;
		float sBackward;
		uint8_t inverse;
	} thrusters[THRUSTERS_NUMBER];
};

#pragma pack(pop)

#define CONFIG_PAGE_ADDR 	0x0803F800
#define FLASH_END_ADDR   	0x08040000		// This is just visual, not used
#define SETTINGS_WORDS 		sizeof(struct flashConfiguration_s)/4

void flashReadSettings(struct flashConfiguration_s *config);
void flashWriteSettings(struct flashConfiguration_s *config);
void flashErase(void);

void flashFillStructure(struct flashConfiguration_s *config);
void flashReadStructure(struct flashConfiguration_s *config);

#endif
