/**
 * @file    definitions.h
 * @author	Jakub Pekar
 * @brief   Súbor obsahújúci definície potrebné pre riadenie výťahu
 * @date	16 .1. 2019
 */

#ifndef INCLUDE_DEFINITIONS_H_
#define INCLUDE_DEFINITIONS_H_
#include <stdint-gcc.h>
/*
 * 		1B				 1B					1B           1B
 * |  StartByte  | ReceiverAddress  | SenderDdress  | DataSize | ... DATA  ... | CRC |
 *
 */

/// Maximálna veľkosť paketu
#define PACKET_SIZE 	261
#define RECV_MSG_SIZE 	9

/// Definíca adresy LED diody 0 vo vnútri výťahu
#define LED_OUT_P 	0x10
/// Definíca adresy LED diody 1 vo vnútri výťahu
#define LED_OUT_1   0x11
/// Definíca adresy LED diody 2 vo vnútri výťahu
#define LED_OUT_2   0x12
/// Definíca adresy LED diody 3 vo vnútri výťahu
#define LED_OUT_3   0x13
/// Definíca adresy LED diody 4 vo vnútri výťahu
#define LED_OUT_4   0x14

/// Definíca adresy vonkajšej LED diody 0 výťahu
#define LED_IN_P 	0x20
/// Definíca adresy vonkajšej LED diody 1 výťahu
#define LED_IN_1   	0x21
/// Definíca adresy vonkajšej LED diody 2 výťahu
#define LED_IN_2   	0x22
/// Definíca adresy vonkajšej LED diody 3 výťahu
#define LED_IN_3   	0x23
/// Definíca adresy vonkajšej LED diody 4 výťahu
#define LED_IN_4   	0x24

/// Definíca adresy senzora na prízemí
#define LIM_SWITCH_P 0xE0
/// Definíca adresy senzora na 1.poschodí
#define LIM_SWITCH_1 0xE1
/// Definíca adresy senzora na 2.poschodí
#define LIM_SWITCH_2 0xE2
/// Definíca adresy senzora na 3.poschodí
#define LIM_SWITCH_3 0xE3
/// Definíca adresy senzora na 4.poschodí
#define LIM_SWITCH_4 0xE4

/// Definíca adresy vnútorného tlačidla pre prízemie
#define BUTT_IN_P 	0xB0
/// Definíca adresy vnútorného tlačidla pre 1. poschodie
#define BUTT_IN_1 	0xB1
/// Definíca adresy vnútorného tlačidla pre 2. poschodie
#define BUTT_IN_2 	0xB2
/// Definíca adresy vnútorného tlačidla pre 3. poschodie
#define BUTT_IN_3 	0xB3
/// Definíca adresy vnútorného tlačidla pre 4. poschodie
#define BUTT_IN_4 	0xB4

/// Definíca adresy vonkajšieho tlačidla na prízemí
#define BUTT_OUT_P 	0xC0
/// Definíca adresy vonkajšieho tlačidla na 1. poschodí
#define BUTT_OUT_1 	0xC1
/// Definíca adresy vonkajšieho tlačidla na 2. poschodí
#define BUTT_OUT_2 	0xC2
/// Definíca adresy vonkajšieho tlačidla na 3. poschodí
#define BUTT_OUT_3 	0xC3
/// Definíca adresy vonkajšieho tlačidla na 4. poschodí
#define BUTT_OUT_4 	0xC4

/// Definíca adresy displeja v kabíne výťahu
#define DISPLAY		0x30
/// Definíca adresy motora výťahu
#define MOTOR		0xf1

/// Definíca adresy kabíny výťahu
#define CABIN		0xF0

/// Definíca adresy terminálu
#define TERMINAL  	0xD0
/// Definíca adresy watchDogu
#define WATCHDOG	0xFE
/// Definíca adresy núdzovej brzdy
#define EMERGENCY_BREAK	0x0f

/**
 * Štruktúra predstavujúca paket ktorý sa posiela/prijíma prostredníctvom UART rozhrania pri komunikácii s výťahom
 */
struct Message{
	/**
	 * Data predstavujúce prijatú správu
	 * 		-[0] Štartovací byte 0XA0
	 * 		-[1] Adresa odosielateľa
	 * 		-[2] Adresa prijímateľa
	 * 		-[3] Veľkosť dát x
	 * 		-[4 - x] Dáta
	 * 		-[x + 5] CRC
	 */
	uint8_t data[15];

	/**
	 * Vracia veľkosť dát správy, bez hlavičky a CRC
	 * @returns veľkosť dát
	 */
	uint8_t getSize()
	{
		return data[3];
	}

	/**
	 * Vracia adresu odosielateľa
	 * @returns adresa odosielateľa
	 */
	uint8_t getSenderAddress()
	{
		return data[2];
	}

	/**
	 * Vracia adresu prijímateľa
	 * @returns adresa prijímateľa
	 */
	uint8_t getReceiverAddress()
	{
		return data[1];
	}

	/**
	 * Vracia hodnotu CRC
	 * @returns CRC
	 */
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
	static volatile bool floorElevator[5] = {false, false, false ,false, false}; // Field that represent floor where has to stop Elevator
	static volatile uint8_t elevatorPosition = 4 ;
	static volatile Direction elevatorDirection;
	static volatile bool elevatorIsMoving = false;



#if defined(__cplusplus)
} //extern C
#endif


#endif /* INCLUDE_DEFINITIONS_H_ */
