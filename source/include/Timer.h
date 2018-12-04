/*
 * Timer.h
 *
 *  Created on: 19. 11. 2018
 *      Author: Jakub Pekar
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "fsl_pit.h"

class Timer {
public:
	Timer();
	~Timer();

	bool starTimer();
	bool setTime(uint64_t us);

private:
	pit_config_t m_config;
};

#endif /* TIMER_H_ */
