/*
 * Controler.cpp
 *
 *  Created on: 22. 12. 2018
 *      Author: jakub
 */

#include <fsl_pit.h>
#include <fsl_port.h>
#include <pin_mux.h>
#include "fsl_debug_console.h"
#include "include/Controler.h"
#include "include/Communicator.h"



QueueHandle_t inputQueue;
QueueHandle_t messageQueue;


Communicator* communicator;
MMA8451Q* accelerometer;


volatile bool hValue = false;
volatile double motorEncoder = 0;

Controler::Controler():_timer()
{
	xTaskCreate(Controler::recvTask,"Receiving_TASK",configMINIMAL_STACK_SIZE + 32, NULL , tskIDLE_PRIORITY + 4,NULL);
//	xTaskCreate(Controler::controlTask1,"CONTROL_TASK_1",configMINIMAL_STACK_SIZE + 32, NULL , tskIDLE_PRIORITY + 2,NULL);
	xTaskCreate(Controler::motorControlTask,"MOTOR_CONTROL",configMINIMAL_STACK_SIZE + 32, NULL , tskIDLE_PRIORITY + 1,NULL);


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
	accelerometer = new MMA8451Q(0x1D);
	_timer.setTime(1000000);

	accelerometer->tapDetection();

	LED_turnOff(BLUE);
	LED_turnOff(RED);
	LED_turnOff(GREEN);

	inputQueue = xQueueCreate(5,sizeof(uint8_t));
	messageQueue = xQueueCreate(5,RECV_MSG_SIZE);

}

Controler::~Controler()
{
	delete accelerometer;
}

void Controler::servedButton(uint8_t address)
{
	switch(address)
	{
		case BUTT_OUT_P:
		case BUTT_IN_P:
			if(!floorElevator[0]){
				communicator->setLed(LED_IN_P, true);
				communicator->setLed(LED_OUT_P, true);
				if(!elevatorIsMoving && elevatorPosition == 0){
					hValue =true;
				}
				floorElevator[0] = true;

			}
			break;
		case BUTT_OUT_1:
		case BUTT_IN_1:
			if(!floorElevator[1]){
				communicator->setLed(LED_IN_1, true);
				communicator->setLed(LED_OUT_1, true);
				floorElevator[1] = true;
			}
			break;
		case BUTT_OUT_2:
		case BUTT_IN_2:
			if(!floorElevator[2]){
				communicator->setLed(LED_IN_2, true);
				communicator->setLed(LED_OUT_2, true);
				floorElevator[2] = true;
			}
			break;
		case BUTT_OUT_3:
		case BUTT_IN_3:
			if(!floorElevator[3]){
				communicator->setLed(LED_IN_3, true);
				communicator->setLed(LED_OUT_3, true);
				floorElevator[3] = true;
			}
			break;
		case BUTT_OUT_4:
		case BUTT_IN_4:
			if(!floorElevator[4]){
				communicator->setLed(LED_IN_4, true);
				communicator->setLed(LED_OUT_4, true);
				floorElevator[4] = true;
			}
			break;

	}
}

void Controler::servedLimitSensor(Message& mess)
{
	switch(mess.getSenderAddress())
	{
		case LIM_SWITCH_P:
			if(floorElevator[0] && elevatorPosition != 0)
			{
				communicator->controlMotor(0, STOP);
				elevatorIsMoving = false;
			}
			elevatorPosition = 0;
			communicator->controlDisplay(elevatorPosition, elevatorDirection);
			break;
		case LIM_SWITCH_1:
			if(floorElevator[1] && elevatorPosition != 1)
			{
				communicator->controlMotor(0, STOP);
				elevatorIsMoving = false;
			}
			elevatorPosition = 1;
			communicator->controlDisplay(elevatorPosition, elevatorDirection);
			break;
		case LIM_SWITCH_2:
			if(floorElevator[2] && elevatorPosition != 2)
			{
				communicator->controlMotor(0, STOP);
				elevatorIsMoving = false;
			}
			elevatorPosition = 2;
			communicator->controlDisplay(elevatorPosition, elevatorDirection);
			break;
		case LIM_SWITCH_3:
			if(floorElevator[3] && elevatorPosition != 3)
			{
				communicator->controlMotor(0, STOP);
				elevatorIsMoving = false;
			}
			elevatorPosition = 3;
			communicator->controlDisplay(elevatorPosition, elevatorDirection);
			break;
		case LIM_SWITCH_4:
			if(floorElevator[4] && elevatorPosition != 4)
			{
				communicator->controlMotor(0, STOP);
				elevatorIsMoving = false;
			}
			elevatorPosition = 4;
			communicator->controlDisplay(elevatorPosition, elevatorDirection);
			break;
	}
}

bool Controler::Run()
{
	_timer.startTimer();
	vTaskStartScheduler();
	return true;
}

void Controler::initElevator()
{
	elevatorPosition = 10;
	uint8_t data = 0x03;
	communicator->sendCommand(0xf1, &data, 1);
	communicator->watchDogReset();
	communicator->emergencyBreak(false);
	communicator->cabineLock(true);
	while(!hValue){
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	if(motorEncoder < -1010)
	{
		floorElevator[1] = true;
		elevatorDirection = UP;
		communicator->controlMotor(30, UP);
		while(elevatorPosition != 1){
				vTaskDelay(300 / portTICK_PERIOD_MS);
		}
		floorElevator[1] = false;
		communicator->controlDisplay(1, STOP);
	}else if (motorEncoder > -950){

		floorElevator[0] = true;
		elevatorDirection = DOWN;
		communicator->controlMotor(30, DOWN);
		while(elevatorPosition != 0){
				vTaskDelay(300 / portTICK_PERIOD_MS);
		}
		floorElevator[0] = false;
		elevatorDirection = UP;
		communicator->controlDisplay(0, STOP);
	}

	communicator->setLed(LED_IN_1, false);
	communicator->setLed(LED_IN_2, false);
	communicator->setLed(LED_IN_3, false);
	communicator->setLed(LED_IN_4, false);
	communicator->setLed(LED_IN_P, false);

	communicator->setLed(LED_OUT_P, false);
	communicator->setLed(LED_OUT_1, false);
	communicator->setLed(LED_OUT_2, false);
	communicator->setLed(LED_OUT_3, false);
	communicator->setLed(LED_OUT_4, false);





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
		//xQueueSendToBackFromISR(inputQueue,(void*) handle->rxData,pdFALSE);
	    rxFinished = true;

	   // PRINTF("RxIdle\n");

	}


}

void Controler::recvTask(void* pvParameters)
{
	uint8_t msgSize;
//	uint8_t counter = 0;
	Message mess;

	for(;;)
	{
		receiver.dataSize = 5;
		receiver.data = mess.data;
		rxFinished = false;
		LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle ,(lpsci_transfer_t*) &receiver,(size_t*) &size);
		//xQueueReceive(inputQueue,(void*) &varValue, portMAX_DELAY);
		while(!rxFinished){
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		msgSize = mess.getSize();
		if(msgSize > 0)
		{
			receiver.dataSize = msgSize;
			receiver.data = mess.data + 5;
			rxFinished = false;
			LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle,(lpsci_transfer_t*) &receiver,(size_t*) &size);
			while(!rxFinished)
			{
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}

		}

		if((mess.data[1] | mess.data[2] | mess.data[3] | mess.data[3]) == 0)
		{
			ackArrived = true;
			LED_switch(BLUE);
		} else {
			if(mess.getSenderAddress() == 0xf1){ //DATA received from motor encoder
				memcpy((void*)&motorEncoder , mess.data + 4, 8);
				hValue = true;
			}
			else
			{
				switch((mess.getSenderAddress() & 0xF0))
				{
					case 0xC0:  //switchs outside of elevator
						servedButton(mess.getSenderAddress());
						break;
					case 0xb0:	//switchs inside of elevator
						servedButton(mess.getSenderAddress());
						break;
					case 0xe0:  //sensors of elevator
						servedLimitSensor(mess);


						break;
					default:
						PRINTF("ERR recv\n");
						break;
				}
			}
		}

	}
}
void Controler::controlTask1(void* pvParameters)
{	bool lock = false;
	uint8_t dt[] = "FromControlTask1\n\r";
	sender.data = dt;
	sender.dataSize = 18;
	Message mess;
	for(;;)
	{
		xQueueReceive(messageQueue, mess.data, portMAX_DELAY);
		lock = (lock ? false:true);
		LED_switch(BLUE);





	}
}

void Controler::motorControlTask(void* pvParameters)
{
	vTaskDelay(100 / portTICK_PERIOD_MS);
	initElevator();
//	elevatorIsMoving = true;
	PRINTF("After INIT\n");
	uint8_t nextFloor = 0;
	bool isNextFloor = false;
	hValue = false;
	Direction elevatorState = STOP;
	for(;;)
	{
		if( !elevatorIsMoving && (floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
		{

			if(elevatorDirection == UP){

				if(isNextFloor || hValue){
					/*door OPEN*/
					vTaskDelay(1500 / portTICK_PERIOD_MS);

					/*Turn of the LEDS */
					communicator->setLed(0x20 + elevatorPosition, false);
					communicator->setLed(0x10 + elevatorPosition, false);
					LED_turnOn(GREEN);
					communicator->controlDisplay(elevatorPosition, STOP);
					communicator->cabineLock(false);
					vTaskDelay(2000 / portTICK_PERIOD_MS);
					communicator->cabineLock(true);
					vTaskDelay(500 / portTICK_PERIOD_MS);
					LED_turnOff(GREEN);
					floorElevator[elevatorPosition] = false;
					hValue = false;
				}

				//next moving of elevator
				if(elevatorPosition == 0 && (floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4] ))
				{
					communicator->controlMotor(20, elevatorDirection);
					communicator->controlDisplay(elevatorPosition, UP);
					elevatorIsMoving = true;
					isNextFloor =true;
				}
				else if((elevatorPosition == 1) && (floorElevator[2] || floorElevator[3] ||floorElevator[4] ))
				{
					communicator->controlMotor(20, elevatorDirection);
					communicator->controlDisplay(elevatorPosition, UP);
					elevatorIsMoving = true;
					isNextFloor =true;
				}
				else if((elevatorPosition == 2) && (floorElevator[3] ||floorElevator[4]) )
				{
					communicator->controlMotor(20, elevatorDirection);
					communicator->controlDisplay(elevatorPosition, UP);
					elevatorIsMoving = true;
					isNextFloor =true;
				}
				else if(elevatorPosition == 3 && floorElevator[4] )
				{
					communicator->controlMotor(20, elevatorDirection);
					communicator->controlDisplay(elevatorPosition, UP);
					elevatorIsMoving = true;
					isNextFloor =false;
				}
				else if((floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
				{

						elevatorDirection = DOWN;
						isNextFloor =false;

				}else{
					elevatorDirection = STOP;
					isNextFloor =false;
				}
			}
			else if (elevatorDirection == DOWN)
			{
				if(isNextFloor || hValue){
					/*Turn of the LEDS */
					communicator->setLed(0x20 + elevatorPosition, false);
					communicator->setLed(0x10 + elevatorPosition, false);

					/*door OPEN*/
					vTaskDelay(1500 / portTICK_PERIOD_MS);
					communicator->setLed(0x20 + elevatorPosition, false);
					communicator->setLed(0x10 + elevatorPosition, false);
					LED_turnOn(GREEN);
					communicator->controlDisplay(elevatorPosition, STOP);
					communicator->cabineLock(false);
					vTaskDelay(2000 / portTICK_PERIOD_MS);
					communicator->cabineLock(true);
					vTaskDelay(500 / portTICK_PERIOD_MS);
					LED_turnOff(GREEN);
					floorElevator[elevatorPosition] = false;
					hValue = false;
				}

				//next moving of elevator
				if(elevatorPosition == 4 && (floorElevator[3] || floorElevator[2] ||floorElevator[1] ||floorElevator[0] ))
				{
					communicator->controlMotor(20, elevatorDirection);
					elevatorIsMoving = true;
					isNextFloor = true;
				}
				else if((elevatorPosition == 3) && (floorElevator[2] || floorElevator[1] ||floorElevator[0] ))
				{
					communicator->controlMotor(20, elevatorDirection);
					elevatorIsMoving = true;
					isNextFloor = true;
				}
				else if((elevatorPosition == 2) && (floorElevator[1] ||floorElevator[0]) )
				{
					communicator->controlMotor(20, elevatorDirection);
					elevatorIsMoving = true;
					isNextFloor = true;
				}
				else if(elevatorPosition == 1 && floorElevator[0] )
				{
					communicator->controlMotor(20, elevatorDirection);
					elevatorIsMoving = true;
					isNextFloor = false;
				}
				else if((floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
				{

						elevatorDirection = UP;
						isNextFloor =false;

				}
				else
				{
					elevatorDirection = STOP;
					isNextFloor =false;
					vTaskDelay(100 / portTICK_PERIOD_MS);
				}

			}
			else
			{
				if((floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
				{
					elevatorDirection = UP;
				}
			}

		}
		else
		{
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	}
}

void PORTA_DriverIRQHandler(void){
//
	uint8_t retStatus;
	accelerometer->readRegs(0x0C, &retStatus, 1);
	if(retStatus == 0x04)
		accelerometer->readRegs(0x16, &retStatus, 1);

	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT2_PIN);
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT1_PIN);

	if(retStatus > 0) {
		communicator->emergencyBreak(true);
		LED_switch(RED);
	}
}



void PIT_IRQHandler()
{

	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	communicator->watchDogHandler();

}

