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


Communicator::Communicator(): _myAddress(0x00)
{
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

	//txFinished = false;
	//LPSCI_TransferSendNonBlocking(UART0, (lpsci_handle_t*) &uart_handle , (lpsci_transfer_t*) &sender);
	LPSCI_WriteBlocking(UART0, sender.data, sender.dataSize);
	return true;
}

bool Communicator::receivingData(Message& message)
{
	receiver.dataSize = 1;
	rxFinished = false;
	LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
	while(!rxFinished); // receiving of start

	// if receiver.data == 0xA0
	receiver.data = message.head;
	receiver.dataSize = 3;
	rxFinished = false;
	LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
	while(!rxFinished); //waiting for receiving head of message


	receiver.data = message.data;
	receiver.dataSize = (message.head[2] +1);
	rxFinished = false;
	LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
	while(!rxFinished); // waiting for receiving of data

	return this->verifyMessage(message); // comparing of CRC

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

void Communicator::writeToConsole(uint8_t* message,uint8_t size)
{
	sendCommand(TERMINAL, message, size);
}

void Communicator::UARTInit()
{
	lpsci_config_t user_config;
	LPSCI_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 57600U;
	LPSCI_Init(UART0, &user_config, CLOCK_GetFreq(kCLOCK_PllFllSelClk));

	LPSCI_EnableTx(UART0, true);
	LPSCI_EnableRx(UART0, true);

	receiver.data = (uint8_t*) &rxChar;
	receiver.dataSize = 1;

	sender.data =(uint8_t*) sendBuffer;
	sender.dataSize = 0;
	LPSCI_TransferCreateHandle(UART0,(lpsci_handle_t*) &uart_handle, &Controler::uart_callback, NULL);
	rxFinished = false;

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

