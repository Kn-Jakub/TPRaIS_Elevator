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

#include "include/LED.h"
#include "include/Communicator.h"
#include "include/Controler.h"

void BOARD_INIT();

int main(void) {

  	/* Init board hardware. */
	BOARD_INIT();
	global_communicator = new Communicator();


//	accelerometer = new MMA8451Q(0X1D);
//	Controler mainControler;
//	accelerometer->tapDetection();
//
//	global_communicator->emergencyBreak(false);


	/* 		TESTING		*/
	global_communicator->emergencyBreak(true);
	global_communicator->emergencyBreak(false);
	global_communicator->cabineLock(true);
	global_communicator->controlMotor(100, DOWN);
	global_communicator->controlMotor(0, STOP);
	global_communicator->controlDisplay(3, DOWN);
    /* Enter an infinite loop, just incrementing a counter. */
    while(1)
    {

    }
    delete global_communicator;
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


// BORDEL
//Timer timer;
//	accelerometer = new MMA8451Q(ACCELL_ADDRESS);
//	accelerometer->freefall();
//
//	timer.setTime((uint64_t)DELTA_T * 1000000);
//	timer.starTimer();
//
//	Communicator communicator;
//
//	uint8_t myData[] = "Elevator are you life \n ";
//	communicator.sendCommand(0Xd0, myData, sizeof(myData));
//    uint8_t comannd = 0x01;
//	communicator.sendCommand(0Xf0, &comannd, 1);
//	communicator.setLed(LED_IN_P, true);
//
//	communicator.setLed(LED_OUT_3, true);
//
