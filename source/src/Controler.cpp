/*
 * Controler.cpp
 *
 *  Created on: 22. 12. 2018
 *      Author: jakub
 */

#include <fsl_pit.h>
#include <fsl_port.h>
#include <pin_mux.h>
#include "../include/Controler.h"

Controler::Controler():_WDTimer()
{
	_WDTimer.setTime(1000000);
	_WDTimer.starTimer();
}
bool Controler::Run()
{
	return true;
}

void PORTA_DriverIRQHandler(void){

	uint8_t retStatus;
	accelerometer->readRegs(0x0C, &retStatus, 1);
	if(retStatus == 0x04)
		accelerometer->readRegs(0x16, &retStatus, 1);

	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT2_PIN);
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT1_PIN);

	if(retStatus > 0) {
		global_communicator->emergencyBreak(true);
	}
}



void PIT_IRQHandler(){

	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	global_communicator->watchDogHandler();
	LED_switch(BLUE);
}

