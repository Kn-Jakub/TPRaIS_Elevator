/**
 * @file    Timer.cpp
 * @author	Jakub Pekar
 * @brief   Súbor obsahújúci zdrojové kódy pre PIT časovač
 * @date 	26. 10. 2018
 */
#include "fsl_debug_console.h"

#include "../include/Timer.h"

/*
 * V konštruktore prebieha inicializácia PIT periférie
 */
Timer::Timer(): m_config()
{
	PIT_GetDefaultConfig(&m_config);
	PIT_Init(PIT, &m_config);
}

Timer::~Timer() {}

/**
 * Metóda pre spustenie časovača. Na začiatku sa povolia prerušenia a následne je spustený samotný časovač.
 * @returns	true/false
 */
bool Timer::startTimer()
{
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT_IRQn);
	PIT_StartTimer(PIT, kPIT_Chnl_0);
	return true;
}

/**
 * Metoda nastavuje interval časovača, v ktorom bude vykonávaná prerušovacia rutina
 * @param time_us	čas v mikrosekundách
 * @returns	true/false
 */

bool Timer::setTime(uint64_t time_us)
{
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(time_us,CLOCK_GetFreq(kCLOCK_BusClk)));
	return true;
}
