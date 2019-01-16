/*
 * Communicator.cpp
 *
 *  Created on: 3. 12. 2018
 *      Author: jakub
 */


#include <include/Communicator.h>
#include "fsl_debug_console.h"
#include <include/LED.h>
#include <assert.h>
#include "include/Controler.h"

static const uint8_t CRCTab[] =
	    {0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

Communicator::Communicator(): _myAddress(0x00)
{
	sender.data =(uint8_t*) sendBuffer;
	sender.dataSize = 0;
}

Communicator::~Communicator()
{
}

bool Communicator::sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize)
{
	uint8_t sizeOfData = 0;

	sender.data[sizeOfData++] = 0xA0;
	sender.data[sizeOfData++] = elementAddress;
	sender.data[sizeOfData++] = _myAddress;
	sender.data[sizeOfData++] = dataSize;

	sender.data[dataSize + sizeOfData] = calcCRC(elementAddress, _myAddress, data, dataSize);
	memcpy(sender.data + sizeOfData, data, dataSize);
	sizeOfData++;
	sender.dataSize = sizeOfData + dataSize;

	while(!txFinished); // waiting for send all data

//	txFinished = false;
	//LPSCI_TransferSendNonBlocking(UART0, (lpsci_handle_t*) &uart_handle , (lpsci_transfer_t*) &sender);
	LPSCI_WriteBlocking(UART0, sender.data, sender.dataSize);
	return true;
}


void Communicator::setLed(uint8_t ledAddress, bool state)
{
	uint8_t data = uint8_t(state);
	sendCommand(ledAddress, &data, 1);
}

void Communicator::emergencyBreak(bool ON)
{
	uint8_t data = uint8_t(ON);
	sendCommand(EMERGENCY_BREAK, &data, 1);
}

void Communicator::controlDisplay(uint8_t floor,Direction direction)
{
	uint8_t data[2];
	switch(direction)
	{
		case UP:
			data[0] = 0x01;
			break;
		case DOWN:
			data[0] = 0x02;
			break;
		default:
			data[0] = 0x00;
			break;
	}

	switch(floor)
	{
		case 0:
			data[1] = 0x50;
			break;
		case 1:
			data[1] = 0x31;
			break;
		case 2:
			data[1] = 0x32;
			break;
		case 3:
			data[1] = 0x33;
			break;
		case 4:
			data[1] = 0x34;
			break;
	}
	sendCommand(DISPLAY, data, 2);
}

void Communicator::cabineLock(bool lock)
{
	uint8_t data = uint8_t(lock);
	sendCommand(CABIN, &data, 1);
}

void Communicator::controlMotor(uint8_t speed, Direction dir)
{
	uint8_t data[5] = {0x02, 0, 0, 0, 0};
	if(dir == UP)
	{
		data[4] = speed;
	}
	else if (dir == DOWN)
	{
		data[1] = 0xFF;
		data[2] = 0xFF;
		data[3] = 0xFF;
		data[4] = (256-speed);
	}
	else
	{
		data[0]=0x01;
		sendCommand(MOTOR, &data[0], 1);
		return;
	}

	sendCommand(MOTOR, data, 5);
}

void Communicator::watchDogHandler()
{
	uint8_t data = 0x02;
	sendCommand(WATCHDOG, &data, 1);
}

void Communicator::watchDogReset()
{
	uint8_t data = 0x01;
	sendCommand(WATCHDOG, &data, 1);
}

void Communicator::writeToConsole(uint8_t* message,uint8_t size)
{
	sendCommand(TERMINAL, message, size);
}

uint8_t Communicator::calcCRC(uint8_t receiverAddress, uint8_t senderAddress, uint8_t* data, uint8_t dataSize)
{
	uint8_t CRC = 0;

	CRC = CRCTab[CRC^receiverAddress];
	CRC = CRCTab[CRC^senderAddress];
	for(int i = 0; i < dataSize; i++)
	{
		CRC = CRCTab[CRC^data[i]];
	}
	return CRC;
}

bool Communicator::verifyMessage(Message& message)
{
	return (calcCRC(message.getReceiverAddress(), message.getSenderAddress(), message.data, message.getSize()) == message.getCRC());
}

