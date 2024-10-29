#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "i2c.h"
#include "motorLib.h"
#include "buttonInterrupt.h"

__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
  __WFI();	// go to Sleep Mode
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
}

