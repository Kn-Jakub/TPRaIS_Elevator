/*
 * Communicator.cpp
 *
 *  Created on: 3. 12. 2018
 *      Author: jakub
 */

#include <fsl_lpsci.h>
#include <include/Communicator.h>
#include "fsl_debug_console.h"
#include <include/LED.h>
Communicator::Communicator(): m_myAddress(0x00)
{
	this->UARTInit();
}

Communicator::~Communicator()
{
}

bool Communicator::sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize)
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

	for(uint8_t i = 0; i < sizeof(sendBuffer); i++){
		PUTCHAR(sendBuffer[i]);
	}
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

void Communicator::writeToConsole(uint8_t* message,uint8_t size)
{
	sendCommand(TERMINAL, message, size);
}

void Communicator::UARTInit()
{
	lpsci_config_t user_config;
	LPSCI_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 115200U;
	user_config.enableTx = true;
	user_config.enableRx = true;

	LPSCI_Init(UART0, &user_config, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
	LPSCI_DisableInterrupts(UART0, kLPSCI_AllInterruptsEnable);

	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable | kLPSCI_RxOverrunInterruptEnable);

	EnableIRQ(UART0_IRQn);
}

void UART0_IRQHandler(void) {
	UART0_Type* base = UART0;
	uint8_t pcBuffer;

	/* If RX overrun. */
	if (UART0_S1_OR_MASK & base->S1) {
		while (UART0_S1_RDRF_MASK & base->S1) {
			(void) base->D;
		}
		LPSCI_ClearStatusFlags(base, kLPSCI_RxOverrunFlag);
	}

	/* Send data register empty and the interrupt is enabled. */
	if ((base->S1 & kLPSCI_TxDataRegEmptyFlag)) {
		int c = 0;
		//int c = bufferRead(&cBuffer, &pcBuffer, 1);
		if (c > 0) {
//			LPSCI_WriteBlocking(UART0, &pcBuffer, 1);
			//UART0->D = pcBuffer;
		} else {
			/* Disable TX register empty interrupt. */
//			base->C2 &= ~UART0_C2_TIE_MASK;
			LPSCI_DisableInterrupts(base, kLPSCI_TxDataRegEmptyInterruptEnable);
		}
		LPSCI_ClearStatusFlags(base, kLPSCI_TxDataRegEmptyFlag);
	}
	/* If RX overrun. */
	if (UART0_S1_OR_MASK & base->S1) {
		while (UART0_S1_RDRF_MASK & base->S1) {
			(void) base->D;
		}

		LPSCI_ClearStatusFlags(base, kLPSCI_RxOverrunFlag);
	}

	/* Receive data register full */
	if ((UART0_S1_RDRF_MASK & base->S1) && (UART0_C2_RIE_MASK & base->C2)) {
		uint8_t rxData;
		static uint8_t size = 0;
		rxData = base->D;
//		if (rxData == '\n' || rxData == '\r') {
//			callBack_struct_pt->callbackHandler(&outBuffer, size,
//					callBack_struct_pt->data);
//			size = 0;
//		} else {
//			size++;
//			if (isalnum(rxData))
//
//				bufferWrite(&outBuffer, &rxData, 1);
//		}
		LED_switch(RED);
	}

}

