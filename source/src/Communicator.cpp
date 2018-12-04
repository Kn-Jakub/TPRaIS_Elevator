/*
 * Communicator.cpp
 *
 *  Created on: 3. 12. 2018
 *      Author: jakub
 */

#include <include/Communicator.h>
#include "fsl_debug_console.h"

Communicator::Communicator(): m_myAddress(0x00)
{


}

Communicator::~Communicator() {
}

bool Communicator::sendComand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize)
{
	uint8_t CRC = 0;
	uint8_t sendBuffer[dataSize + 5];
	sendBuffer[0] = 0xA0;
	sendBuffer[1] = elementAddress;
	sendBuffer[2] = m_myAddress;
	sendBuffer[3] = dataSize;

	CRC = CRCTab[CRC^elementAddress];
	CRC = CRCTab[CRC^m_myAddress];
	for(int i = 0; i < dataSize; i++)
	{
		CRC = CRCTab[CRC^data[i]];
		sendBuffer[i+4] = data[i];
	}
	sendBuffer[dataSize + 4] = CRC;
	//PRINTF("%s",sendBuffer);
	for(int i = 0; i < sizeof(sendBuffer); i++){
		PUTCHAR(sendBuffer[i]);
	}



	return true;
}

