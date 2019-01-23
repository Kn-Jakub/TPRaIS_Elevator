/*
 * @file    Controler.h
 * @author	Jakub Pekar
 * @brief   Súbor obsahujúci deklaráciu triedy Controler
 * @date 	3. 12. 2018
 */

/*
 * 		1B				 1B					1B           1B
 * |  StartByte  | ReceiverAddress  | SenderDdress  | DataSize | ... DATA  ... | CRC |
 *
 *
 */

#ifndef SRC_COMMUNICATOR_H_
#define SRC_COMMUNICATOR_H_

#include <stdint-gcc.h>
#include <fsl_lpsci.h>
#include "include/definitions.h"


/**
 * Trieda zabaľujúca komunikáciu s výťahom
 */

class Communicator {
public:
	Communicator(lpsci_handle_t* _uart_handle);
	virtual ~Communicator() = default;

	bool sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize);
	void setLed(uint8_t ledAddress, bool state);

	void emergencyBreak(bool ON);
	void controlDisplay(uint8_t floor,Direction direction);
	void cabineLock(bool lock);
	void controlMotor(uint8_t speed, Direction dir);
	void watchDogHandler();
	void watchDogReset();
	void writeToConsole(uint8_t* message,uint8_t size);
	bool verifyMessage(Message& message);
	uint8_t calcCRC(uint8_t receiverAddress, uint8_t senderAddress, uint8_t* data, uint8_t dataSize);

private:
	uint8_t _myAddress;
	lpsci_handle_t* _uart_handle;

};

#endif /* SRC_COMMUNICATOR_H_ */
