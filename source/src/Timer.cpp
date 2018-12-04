/*
 * Timer.cpp
 *
 *  Created on: 19. 11. 2018
 *      Author: Jakub Pekar
 */

#include "fsl_debug_console.h"

#include "../include/Timer.h"

Timer::Timer(): m_config()
{
	PIT_GetDefaultConfig(&m_config);
	PIT_Init(PIT, &m_config);
}

Timer::~Timer() {}

bool Timer::starTimer()
{
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT_IRQn);
	PIT_StartTimer(PIT, kPIT_Chnl_0);
	return true;
}

bool Timer::setTime(uint64_t time_us)
{
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(time_us,CLOCK_GetFreq(kCLOCK_BusClk)));
	return true;
}
