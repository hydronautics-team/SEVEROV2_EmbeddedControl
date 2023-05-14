#ifndef FREERTOSTICK_H_
#define FREERTOSTICK_H_

#include "cmsis_os.h"
#include "FreeRTOSConfig.h"

TickType_t fromMsToTick(float ms);
TickType_t fromSecToTick(float sec);
float fromTickToMs(TickType_t ticks);
float fromTickToSec(TickType_t ticks);


#endif /* FREERTOSTICK_H_ */
