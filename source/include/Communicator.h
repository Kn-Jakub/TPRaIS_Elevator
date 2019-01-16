/*
 * Communicator.h
 *
 *  Created on: 3. 12. 2018
 *      Author: Jakub Pekar
 */

/*
 * 		1B				 1B					1B           1B
 * |  StartByte  | ReceiverAddress  | SenderDdress  | DataSize | ... DATA  ... | CRC |
 *
 *
 */

#ifndef SRC_COMMUNICATOR_H_
#define SRC_COMMUNICATOR_H_
//#include "include/RingBuffer.h"
#include <stdint-gcc.h>
#include <fsl_lpsci.h>
#include "include/definitions.h"




class Communicator {
public:
	Communicator();
	virtual ~Communicator();

	bool sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize);
	void setLed(uint8_t ledAddress, bool state);

	void emergencyBreak(bool ON);
	void controlDisplay(uint8_t floor,Direction direction);
	void cabineLock(bool lock);
	void controlMotor(uint8_t speed, Direction dir);
	void watchDogHandler();
	void writeToConsole(uint8_t* message,uint8_t size);
	bool verifyMessage(Message& message);
	uint8_t calcCRC(uint8_t receiverAddress, uint8_t senderAddress, uint8_t* data, uint8_t dataSize);
private:


private:
	uint8_t _myAddress;

};

#endif /* SRC_COMMUNICATOR_H_ */
