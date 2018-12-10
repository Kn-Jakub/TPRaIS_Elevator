/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @brief   Main application for accelerometer
 * @author	Jakub Pekár
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include <fsl_port.h>

#include "include/MMA8451Q.h"
#include "include/LED.h"
#include "include/Timer.h"
#include "include/Communicator.h"

#define ACCELL_ADDRESS 		0x1DU
#define DELTA_T (float)		1
#define BOARD_DEBUG_UART_BAUDRATE 57600

#if defined(__cplusplus)
extern "C" {
#endif
	volatile int flagIRQ = 0;

	void PIT_IRQHandler();
	void PORTA_DriverIRQHandler(void);


} //extern C

void BOARD_INIT();

MMA8451Q* accelerometer;

int main(void) {

  	/* Init board hardware. */
	BOARD_INIT();
	Timer timer;
	accelerometer = new MMA8451Q(ACCELL_ADDRESS);
	accelerometer->freefall();

	timer.setTime((uint64_t)DELTA_T * 1000000);
	timer.starTimer();

	Communicator communicator;

	uint8_t myData[] = "Elevator are you life \n ";
	communicator.sendComand(0Xd0, myData, sizeof(myData));
    uint8_t comannd = 0x01;
	communicator.sendComand(0Xf0, &comannd, 1);

    /* Enter an infinite loop, just incrementing a counter. */
    while(1)
    {
    	if(flagIRQ == 1){
    		LED_switch(BLUE);
    		communicator.sendComand(0Xd0, myData, sizeof(myData));
    		flagIRQ = 0;
    	}

    }

    return 0 ;
}

void BOARD_INIT()
{
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
}

void PORTA_DriverIRQHandler(void){

	LED_turnOn(BLUE);
	uint8_t retStatus;
	accelerometer->readRegs(0x0C, &retStatus, 1);
	if(retStatus == 0x04)
		accelerometer->readRegs(0x16, &retStatus, 1);

	uint32_t interFlags = PORT_GetPinsInterruptFlags(PORTA);
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT2_PIN);
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT1_PIN);
	interFlags = PORT_GetPinsInterruptFlags(PORTA);
	if(retStatus > 0) {
		PRINTF("FREE FALL [retValue: %d]!!!!!!\n\r", retStatus);
	} else {
		//PRINTF("Something Wrong [%d]\n\r", retStatus);

	}
};



void PIT_IRQHandler(){

	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	flagIRQ = 1;
	//PRINTF("PIT handler\r\n");
}
