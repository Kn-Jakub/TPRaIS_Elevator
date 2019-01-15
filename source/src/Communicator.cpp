/*
 * Communicator.cpp
 *
 *  Created on: 3. 12. 2018
 *      Author: jakub
 */


#include <include/Communicator.h>
#include "fsl_debug_console.h"
#include <include/LED.h>

void Communicator::uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
	userData = userData;
	if (kStatus_LPSCI_TxIdle == status)
	{
		txFinished = true;
        PRINTF("TxIdle\n");
	}
	if (kStatus_LPSCI_RxIdle == status)
	{
	    rxFinished = true;
	    PRINTF("RxIdle\n");
	    LPSCI_TransferReceiveNonBlocking(UART0, handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
	}


}

Communicator::Communicator(RingBuffer* buffer): _buffer(buffer), _myAddress(0x00)
{
	this->UARTInit();
}

Communicator::~Communicator()
{
}

bool Communicator::sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize)
{
	uint8_t sizeOfData = 0;

//	sendBuffer[0] = 0xA0;
//	sendBuffer[1] = elementAddress;
//	sendBuffer[2] = _myAddress;
//	sendBuffer[3] = dataSize;
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

	LPSCI_TransferCreateHandle(UART0,(lpsci_handle_t*) &uart_handle, Communicator::uart_callback, NULL);
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

//void UART0_IRQHandler(void) {
//	UART0_Type* base = UART0;
//	uint8_t pcBuffer;
//
//	/* If RX overrun. */
//	if (UART0_S1_OR_MASK & base->S1) {
//		while (UART0_S1_RDRF_MASK & base->S1) {
//			(void) base->D;
//		}
//		LPSCI_ClearStatusFlags(base, kLPSCI_RxOverrunFlag);
//	}
//
//	/* Send data register empty and the interrupt is enabled. */
//	if ((base->S1 & kLPSCI_TxDataRegEmptyFlag)) {
//		int c = 0;
//		//int c = bufferRead(&cBuffer, &pcBuffer, 1);
//		if (c > 0) {
////			LPSCI_WriteBlocking(UART0, &pcBuffer, 1);
//			//UART0->D = pcBuffer;
//		} else {
//			/* Disable TX register empty interrupt. */
////			base->C2 &= ~UART0_C2_TIE_MASK;
//			LPSCI_DisableInterrupts(base, kLPSCI_TxDataRegEmptyInterruptEnable);
//		}
//		LPSCI_ClearStatusFlags(base, kLPSCI_TxDataRegEmptyFlag);
//	}
//	/* If RX overrun. */
//	if (UART0_S1_OR_MASK & base->S1) {
//		while (UART0_S1_RDRF_MASK & base->S1) {
//			(void) base->D;
//		}
//
//		LPSCI_ClearStatusFlags(base, kLPSCI_RxOverrunFlag);
//	}
//
//	/* Receive data register full */
//	if ((UART0_S1_RDRF_MASK & base->S1) && (UART0_C2_RIE_MASK & base->C2)) {
//		uint8_t rxData;
//		static uint8_t size = 0;
//		rxData = base->D;
//		//global_buffer->push(&rxData, 1);
//		PUTCHAR(rxData);
////		if (rxData == '\n' || rxData == '\r') {
////			callBack_struct_pt->callbackHandler(&outBuffer, size,
////					callBack_struct_pt->data);
////			size = 0;
////		} else {
////			size++;
////			if (isalnum(rxData))
////
////				bufferWrite(&outBuffer, &rxData, 1);
////		}
//		LED_switch(RED);
//	}
//
//}

