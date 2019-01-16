/*
 * Controler.cpp
 *
 *  Created on: 22. 12. 2018
 *      Author: jakub
 */

#include <fsl_pit.h>
#include <fsl_port.h>
#include <pin_mux.h>
#include "include/Controler.h"
#include "fsl_debug_console.h"
#include "include/Communicator.h"
#include "include/definitions.h"

QueueHandle_t inputQueue;
QueueHandle_t messageQueue;


Communicator* communicator;

//static Communicator* communicator;
Controler::Controler()
{
	//callbackHandlerUART = uart_callback;
	xTaskCreate(Controler::recvTask,"Receiving_TASK",configMINIMAL_STACK_SIZE + 64, NULL , tskIDLE_PRIORITY + 2,NULL);
	xTaskCreate(Controler::controlTask1,"CONTROL_TASK_1",configMINIMAL_STACK_SIZE + 64, NULL , tskIDLE_PRIORITY + 2,NULL);

	lpsci_config_t user_config;
	LPSCI_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 57600U;
	LPSCI_Init(UART0, &user_config, CLOCK_GetFreq(kCLOCK_PllFllSelClk));

	LPSCI_EnableTx(UART0, true);
	LPSCI_EnableRx(UART0, true);

	LPSCI_TransferCreateHandle(UART0, (lpsci_handle_t*) &uart_handle, &uart_callback, NULL);
	LPSCI_TransferStartRingBuffer(UART0, (lpsci_handle_t*) &uart_handle, (uint8_t*)ringBuffer, PACKET_SIZE);

	receiver.data =(uint8_t*) rxCommand;
	receiver.dataSize = 6;

	communicator = new Communicator();

	inputQueue = xQueueCreate(5,sizeof(uint8_t));
	messageQueue = xQueueCreate(5,RECV_MSG_SIZE);
//	communicator = new Communicator();
//	communicator->UARTInit();

}
bool Controler::Run()
{
	vTaskStartScheduler();
	return true;
}

void uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
	if (kStatus_LPSCI_TxIdle == status)
	{

        PRINTF("TxIdle\n");
        txFinished = true;
	}
	if (kStatus_LPSCI_RxIdle == status)
	{
		//
		xQueueSendToBackFromISR(inputQueue,(void*) handle->rxData,pdFALSE);
	    rxFinished = true;

	   // PRINTF("RxIdle\n");

	}


}

void Controler::recvTask(void* pvParameters)
{
//	uint8_t varValue;
//	uint8_t counter = 0;
	Message mess;


	uint8_t dt[] = "FromControlTask1\n\r";
		sender.data = dt;
		sender.dataSize = 18;
		LPSCI_TransferSendNonBlocking(UART0, (lpsci_handle_t*) &uart_handle , (lpsci_transfer_t*) &sender);


	for(;;)
	{
		receiver.dataSize = 4;
		receiver.data = mess.data;
		rxFinished = false;
		LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle ,(lpsci_transfer_t*) &receiver,(size_t*) &size);
		//xQueueReceive(inputQueue,(void*) &varValue, portMAX_DELAY);
		while(!rxFinished){
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}

		receiver.dataSize = mess.getSize() +1;
		receiver.data = mess.data + 4;
		rxFinished = false;
		LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
		while(!rxFinished){
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}

		if((mess.data[1] | mess.data[2] | mess.data[3]) == 0)
		{
			ackArrived = true;
		} else {
			xQueueSendToBack(messageQueue,mess.data,pdFALSE);
		}
		PRINTF("%s", mess.data);

	}
}
void Controler::controlTask1(void* pvParameters)
{	bool lock = false;
	uint8_t dt[] = "FromControlTask1\n\r";
	sender.data = dt;
	sender.dataSize = 18;
	Message mess;
	communicator->controlMotor(50, DOWN);
	for(;;)
	{
		xQueueReceive(messageQueue, mess.data, portMAX_DELAY);
		lock = (lock ? false:true);
		LED_switch(BLUE);

		switch((mess.getSenderAddress() & 0xF0))
		{
			case 0xC0:  //switchs outside of elevator
				communicator->setLed(LED_OUT_4, lock);
				break;
			case 0xb0:	//switchs inside of elevator
				communicator->setLed(LED_IN_4, lock);
				break;
			case 0xe0:  //senzors of elevator
				if(mess.getSenderAddress() == LIM_SWITCH_1){
					communicator->controlMotor(0, STOP);
				}

				break;
			default:
				communicator->cabineLock(lock);
				break;
		}



	}
}

void PORTA_DriverIRQHandler(void){
//
//	uint8_t retStatus;
//	accelerometer->readRegs(0x0C, &retStatus, 1);
//	if(retStatus == 0x04)
//		accelerometer->readRegs(0x16, &retStatus, 1);
//
//	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT2_PIN);
//	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT1_PIN);
//
//	if(retStatus > 0) {
//		global_communicator->emergencyBreak(true);
//	}
}



void PIT_IRQHandler(){

//	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
//	global_communicator->watchDogHandler();
//	LED_switch(BLUE);
}

