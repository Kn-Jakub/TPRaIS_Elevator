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

static QueueHandle_t inputQueue;
static Communicator* communicator;
Controler::Controler()
{
	//callbackHandlerUART = uart_callback;
	xTaskCreate(Controler::recvTask,"Receiving_TASK",configMINIMAL_STACK_SIZE + 64, NULL , tskIDLE_PRIORITY + 2,NULL);
	xTaskCreate(Controler::controlTask1,"CONTROL_TASK_1",configMINIMAL_STACK_SIZE + 64, NULL , tskIDLE_PRIORITY + 2,NULL);
	inputQueue = xQueueCreate(32,sizeof(uint8_t));
	communicator = new Communicator();
	communicator->UARTInit();

}
bool Controler::Run()
{
	vTaskStartScheduler();
	return true;
}

void Controler::uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
	if (kStatus_LPSCI_TxIdle == status)
	{
		xQueueSendToBackFromISR(inputQueue,(void*) &rxChar,pdFALSE);
        PRINTF("TxIdle\n");
	}
	if (kStatus_LPSCI_RxIdle == status)
	{
	    rxFinished = true;
	    PRINTF("RxIdle\n");
	    LPSCI_TransferReceiveNonBlocking(UART0, handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
	}


}

void Controler::recvTask(void* pvParameters)
{
	uint8_t varValue;
	bool messageReceiving = false;
	for(;;)
	{
		xQueueReceive(inputQueue, &varValue, portMAX_DELAY);
		PRINTF("rcv: ");
		PRINTF("%c\n",varValue);
		if(messageReceiving){

		}else{
			if(varValue == 0xA0){

			}
		}
	}
}
void Controler::controlTask1(void* pvParameters)
{	bool lock = false;
	for(;;)
	{	lock = (lock ? false:true);
		LED_switch(BLUE);
		communicator->cabineLock(lock);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
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

