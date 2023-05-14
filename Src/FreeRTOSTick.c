#include "FreeRTOSTick.h"

uint32_t fromMsToTick(float ms)
{
	return (ms * configTICK_RATE_HZ) / 1000;
}

uint32_t fromSecToTick(float sec) {
	return (sec * configTICK_RATE_HZ);
}

float fromTickToMs(uint32_t ticks) {
	return ((float) (ticks) * 1000.0f) / (float) configTICK_RATE_HZ;
}

float fromTickToSec(uint32_t ticks) {
	return ((float) ticks) / configTICK_RATE_HZ;
}
