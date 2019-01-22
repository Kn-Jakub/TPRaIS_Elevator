/*
 * definitions.h
 *
 *  Created on: 16. 1. 2019
 *      Author: jakub
 */

#ifndef INCLUDE_DEFINITIONS_H_
#define INCLUDE_DEFINITIONS_H_
#include <stdint-gcc.h>
/*
 * 		1B				 1B					1B           1B
 * |  StartByte  | ReceiverAddress  | SenderDdress  | DataSize | ... DATA  ... | CRC |
 *
 */

#define PACKET_SIZE 	261
#define RECV_MSG_SIZE 	9

#define LED_OUT_P 	0x10
#define LED_OUT_1   0x11
#define LED_OUT_2   0x12
#define LED_OUT_3   0x13
#define LED_OUT_4   0x14

#define LED_IN_P 	0x20
#define LED_IN_1   	0x21
#define LED_IN_2   	0x22
#define LED_IN_3   	0x23
#define LED_IN_4   	0x24

#define LIM_SWITCH_P 0xE0
#define LIM_SWITCH_1 0xE1
#define LIM_SWITCH_2 0xE2
#define LIM_SWITCH_3 0xE3
#define LIM_SWITCH_4 0xE4

#define BUTT_IN_P 	0xB0
#define BUTT_IN_1 	0xB1
#define BUTT_IN_2 	0xB2
#define BUTT_IN_3 	0xB3
#define BUTT_IN_4 	0xB4

#define BUTT_OUT_P 	0xC0
#define BUTT_OUT_1 	0xC1
#define BUTT_OUT_2 	0xC2
#define BUTT_OUT_3 	0xC3
#define BUTT_OUT_4 	0xC4

#define DISPLAY		0x30
#define MOTOR		0xf1

#define CABIN		0xF0

#define TERMINAL  	0xd0
#define WATCHDOG	0xfe
#define EMERGENCY_BREAK	0x0f

struct Message{
	uint8_t data[15];

	uint8_t getSize()
	{
		return data[3];
	}

	uint8_t getSenderAddress()
	{
		return data[2];
	}

	uint8_t getReceiverAddress()
	{
		return data[1];
	}

	uint8_t getCRC()
	{
		return data[this->getSize()+1];
	}
};

enum Direction{
	UP = 0,
	DOWN = 1,
	STOP

};


#if defined(__cplusplus)
extern "C" {
#endif
	//void UART0_IRQHandler(void);
	static volatile uint8_t sendBuffer[PACKET_SIZE];
	static volatile uint8_t ringBuffer[PACKET_SIZE];
	static volatile uint8_t rxChar;
	static volatile uint8_t rxCommand[20];
	static volatile size_t size = 1;

	static volatile lpsci_transfer_t sender;
	static volatile lpsci_transfer_t receiver;

	static volatile lpsci_handle_t uart_handle;

	static volatile bool txFinished = true;
	static volatile bool rxFinished = true;
	static volatile bool ackArrived = false;

	/* the global variables for observing elevator*/
	static volatile bool floorElevator[5] = {false}; // Field that represent floor where has to stop Elevator
	static volatile uint8_t elevatorPosition = 4 ;
	static volatile Direction elevatorDirection;
	static volatile bool elevatorIsMoving = false;



#if defined(__cplusplus)
} //extern C
#endif


#endif /* INCLUDE_DEFINITIONS_H_ */
