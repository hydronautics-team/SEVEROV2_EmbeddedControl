#include "flash.h"

#include "global.h"

void flashErase()
{
	HAL_FLASH_Unlock();
	// erase page
	FLASH_EraseInitTypeDef erase_conf;
	erase_conf.TypeErase = FLASH_TYPEERASE_SECTORS; // erase 1 page
	erase_conf.Sector = (uint32_t)(CONFIG_PAGE_ADDR);
	erase_conf.NbSectors = 1;

	uint32_t page_error;
	HAL_FLASHEx_Erase(&erase_conf, &page_error);
	HAL_FLASH_Lock();
}

void flashReadSettings(struct flashConfiguration_s *config)
{
	uint32_t *source_addr = (uint32_t *)CONFIG_PAGE_ADDR;
	uint32_t *dest_addr = (void *)config;

	for (uint16_t i = 0; i < SETTINGS_WORDS; ++i) {
		*dest_addr = *(__IO uint32_t*)source_addr;
		source_addr++;
		dest_addr++;
	}
}

void flashWriteSettings(struct flashConfiguration_s *config)
{
	// Write settings
	HAL_FLASH_Unlock();

	// erase page
	FLASH_EraseInitTypeDef erase_conf;
	erase_conf.TypeErase = FLASH_TYPEERASE_SECTORS; // erase 1 page
	erase_conf.Sector = (uint32_t)(CONFIG_PAGE_ADDR);
	erase_conf.NbSectors = 1;

	uint32_t page_error;
	HAL_FLASHEx_Erase(&erase_conf, &page_error);

	// write page
	uint32_t *source_addr = (void *)config;
	uint32_t *dest_addr = (uint32_t *)CONFIG_PAGE_ADDR;
	for (uint8_t i = 0; i < SETTINGS_WORDS; ++i) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr);
		source_addr++;
		dest_addr++;
	}

	HAL_FLASH_Lock();
}

void flashFillStructure(struct flashConfiguration_s *config)
{
	config->writeFlag = 0xAA;

	for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
		config->stabConstants[i].pJoyUnitCast = rStabConstants[i].pJoyUnitCast;
		config->stabConstants[i].pSpeedDyn = rStabConstants[i].pSpeedDyn;
		config->stabConstants[i].pErrGain = rStabConstants[i].pErrGain;
		config->stabConstants[i].aFilter_T1 = rStabConstants[i].aFilter[POS_FILTER].T;
		config->stabConstants[i].aFilter_T2 = rStabConstants[i].aFilter[SPEED_FILTER].T;
		config->stabConstants[i].aFilter_K1 = rStabConstants[i].aFilter[POS_FILTER].K;
		config->stabConstants[i].aFilter_K2 = rStabConstants[i].aFilter[SPEED_FILTER].K;
		config->stabConstants[i].pid_pGain = rStabConstants[i].pid.pGain;
		config->stabConstants[i].pid_iGain = rStabConstants[i].pid.iGain;
		config->stabConstants[i].pid_iMax = rStabConstants[i].pid.iMax;
		config->stabConstants[i].pid_iMin = rStabConstants[i].pid.iMin;
		config->stabConstants[i].pThrustersMin = rStabConstants[i].pThrustersMin;
		config->stabConstants[i].pThrustersMax = rStabConstants[i].pThrustersMax;
		config->stabConstants[i].aFilter_thrusters_T = rStabConstants[i].aFilter[THRUSTERS_FILTER].T;
		config->stabConstants[i].aFilter_thrusters_K = rStabConstants[i].aFilter[THRUSTERS_FILTER].K;
		config->stabConstants[i].sOutSummatorMax = rStabConstants[i].sOutSummatorMax;
		config->stabConstants[i].sOutSummatorMin = rStabConstants[i].sOutSummatorMin;
	}

//	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
//		config->thrusters[i].address = rThrusters[i].address;
//		config->thrusters[i].kForward = rThrusters[i].kForward;
//		config->thrusters[i].kBackward = rThrusters[i].kBackward;
//		config->thrusters[i].sForward = rThrusters[i].sForward;
//		config->thrusters[i].sBackward = rThrusters[i].sBackward;
//		config->thrusters[i].inverse = rThrusters[i].inverse;
//	}
}

void flashReadStructure(struct flashConfiguration_s *config)
{
	for(uint8_t i=0; i<STABILIZATION_AMOUNT; i++) {
		rStabConstants[i].pJoyUnitCast = config->stabConstants[i].pJoyUnitCast;
		rStabConstants[i].pSpeedDyn = config->stabConstants[i].pSpeedDyn;
		rStabConstants[i].pErrGain = config->stabConstants[i].pErrGain;
		rStabConstants[i].aFilter[POS_FILTER].T = config->stabConstants[i].aFilter_T1;
		rStabConstants[i].aFilter[SPEED_FILTER].T = config->stabConstants[i].aFilter_T2;
		rStabConstants[i].aFilter[POS_FILTER].K = config->stabConstants[i].aFilter_K1;
		rStabConstants[i].aFilter[SPEED_FILTER].K = config->stabConstants[i].aFilter_K2;
		rStabConstants[i].pid.pGain = config->stabConstants[i].pid_pGain;
		rStabConstants[i].pid.iGain = config->stabConstants[i].pid_iGain;
		rStabConstants[i].pid.iMax = config->stabConstants[i].pid_iMax;
		rStabConstants[i].pid.iMin = config->stabConstants[i].pid_iMin;
		rStabConstants[i].pThrustersMin = config->stabConstants[i].pThrustersMin;
		rStabConstants[i].pThrustersMax = config->stabConstants[i].pThrustersMax;
		rStabConstants[i].aFilter[THRUSTERS_FILTER].T = config->stabConstants[i].aFilter_thrusters_T;
		rStabConstants[i].aFilter[THRUSTERS_FILTER].K = config->stabConstants[i].aFilter_thrusters_K;
		rStabConstants[i].sOutSummatorMax = config->stabConstants[i].sOutSummatorMax;
		rStabConstants[i].sOutSummatorMin = config->stabConstants[i].sOutSummatorMin;
	}

//	for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
//		rThrusters[i].address = config->thrusters[i].address;
//		rThrusters[i].kForward = config->thrusters[i].kForward;
//		rThrusters[i].kBackward = config->thrusters[i].kBackward;
//		rThrusters[i].sForward = config->thrusters[i].sForward;
//		rThrusters[i].sBackward = config->thrusters[i].sBackward;
//		rThrusters[i].inverse = config->thrusters[i].inverse;
//	}

	if(config->writeFlag == 0xAA) {
		rState.flash = true;
	}
	else {
		rState.flash = false;
	}
}
