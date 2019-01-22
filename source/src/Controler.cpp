/**
 * @file    Controler.cpp
 * @author	Jakub Pekar
 * @brief   Súbor obsahújúci zdrojové kódy hlavného riadiaceho objektu
 */

#include <fsl_pit.h>
#include <fsl_port.h>
#include <pin_mux.h>
#include "fsl_debug_console.h"

/* API Includes*/
#include "include/Controler.h"
#include "include/Communicator.h"

/**
 * Globálne premenné danej triedy.
 * Musia byť globálne kvoli využívaniu v prerušovacích rutinách
 **/
Communicator* communicator;
MMA8451Q* accelerometer;

volatile bool hValue = false;
volatile double motorEncoder = 0;
/**
 * Konštruktor triedy Controler.
 * Inicializuje atribut _timer, ktorý slúži pre odosielanie správ WatchDog modulu výťahu.
 * Vytvára vlákna v ktorých sa vykonáva obsluha výťahu.
 * Konfiguruje komunikačné rozhranie UART prostredníctvom LPSCI ovládača.
 * Vytvára globálne objekty: 	- communicator - pre komunikáciu s výťahom
 * 								- acceleromter - pre detekciu voľného pádu/detekciu dotyku
 *
 **/
Controler::Controler():_timer()
{
	xTaskCreate(Controler::recvTask,"Receiving_TASK",configMINIMAL_STACK_SIZE + 32, NULL , tskIDLE_PRIORITY + 4,NULL);
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

	communicator = new Communicator(NULL);
	accelerometer = new MMA8451Q(0x1D);

	_timer.setTime(1000000);

	LED_turnOff(BLUE);
	LED_turnOff(RED);
	LED_turnOff(GREEN);
}
/**
 * Deštruktor objektu Controler, v ktorom sú mazané globalné premenné communicator a accelerometer vytvárané dynamicky.
 */
Controler::~Controler()
{
	delete communicator;
	delete accelerometer;
}

/**
 * Funkcia spúšťa timer odosielania správ WatchDog systému, konfiguruje accelerometer na detekciu dotyku (tapDetection) a spúšťa obsluhu definovaných vlákien.
 */
bool Controler::Run()
{
	_timer.startTimer();
	accelerometer->tapDetection();
	vTaskStartScheduler();
	return true;
}
/**
 * Inicializácia výťahu.
 * Vo funkcii sa nadviaže komunikácia so simulátorom výťahu. Odošle sa príkaz pre zistenie aktuálnej pozícii výťahu prostredníctvom motorového enkódera, vyresetuje sa Watchdog a odistí núdzová brzda.
 * Následne sa čaká na prijatie správy z enkódera o polohe výťahu.
 * Na základe zistenej hodnoty sa vyhodnotí operácia s výťahom, v prípade potreby zíde výťah na prízemie.
 * Funkcia vypne všetky LED diody výťahu.
 */
void Controler::initElevator()
{
	elevatorPosition = 10;
	uint8_t data = 0x03;

	communicator->sendCommand(0xf1, &data, 1);

	communicator->watchDogReset();
	communicator->emergencyBreak(false);
	communicator->cabineLock(true);

	/* Čakanie na prijatie stavu enkódera motora */
	while(!hValue){
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	if(motorEncoder < -1025)
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
	else
	{
		elevatorPosition = 0;
	}

	communicator->setLed(LED_IN_P, false);
	communicator->setLed(LED_IN_1, false);
	communicator->setLed(LED_IN_2, false);
	communicator->setLed(LED_IN_3, false);
	communicator->setLed(LED_IN_4, false);

	communicator->setLed(LED_OUT_P, false);
	communicator->setLed(LED_OUT_1, false);
	communicator->setLed(LED_OUT_2, false);
	communicator->setLed(LED_OUT_3, false);
	communicator->setLed(LED_OUT_4, false);

}

/**
 * Obsluha tlačidiel. Funkcia je vyvolaná po stlačení tlačidla. Na základe získanej adresy sa vykoná sekvencia príkazov,
 * ktorá obsahuje zapnutie príslušných LED diód označujúcich poschodia na ktorých výťah bude stáť. Taktiež sa nastavia globálne hodnoty na základe ktorých prebieha riadenie motora.
 * @param: address - adresa prvku výťahu.
 */
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

				if(!elevatorIsMoving && elevatorPosition == 1){
					hValue =true;
				}
				floorElevator[1] = true;
			}
			break;

		case BUTT_OUT_2:
		case BUTT_IN_2:
			if(!floorElevator[2]){
				communicator->setLed(LED_IN_2, true);
				communicator->setLed(LED_OUT_2, true);

				if(!elevatorIsMoving && elevatorPosition == 2){
					hValue =true;
				}
				floorElevator[2] = true;
			}
			break;

		case BUTT_OUT_3:
		case BUTT_IN_3:
			if(!floorElevator[3]){
				communicator->setLed(LED_IN_3, true);
				communicator->setLed(LED_OUT_3, true);

				if(!elevatorIsMoving && elevatorPosition == 3){
					hValue =true;
				}
				floorElevator[3] = true;
			}
			break;

		case BUTT_OUT_4:
		case BUTT_IN_4:
			if(!floorElevator[4]){
				communicator->setLed(LED_IN_4, true);
				communicator->setLed(LED_OUT_4, true);

				if(!elevatorIsMoving && elevatorPosition == 4){
					hValue =true;
				}
				floorElevator[4] = true;
			}
			break;

	}
}

/**
 * Obsluha sensorov výťahu. Sensory detekujú na ktorom poschodí sa aktuálne výťah nachádza.
 * Na základe prijatej adressy sa vykoná obsluha pre daný senzor. Ak je v globálnej premennej nastavené dane poschodie ako poschodie na ktorom jhe potrebné zastaviť, odošle sa správa pre motor výťahu s príkazom zastaviť (STOP).
 * Následne sa aktualizuje displej výťahu.
 * @param: mess - správa z daného senzoru, ktorá obsahuje aj adresu senzora.
 */
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

/**
 * Funkcia obsluhy otvorenia dverí po zastavení na niektorom poschodí.
 * Vo funkcii sa nastavujú LED diódy pre dané poschodie. otvoria sa dvere, počka sa určitý čas a dvere sa opäť zatvoria.
 */
void Controler::openDoor(uint8_t elevPos)
{
	vTaskDelay(1500 / portTICK_PERIOD_MS);

	/* Turn off the LEDs */
	communicator->setLed(0x20 + elevatorPosition, false);
	communicator->setLed(0x10 + elevatorPosition, false);

	/* Turn on hardware LED */
	LED_turnOn(GREEN);

	communicator->controlDisplay(elevatorPosition, STOP);

	communicator->cabineLock(false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	communicator->cabineLock(true);
	vTaskDelay(500 / portTICK_PERIOD_MS);

	/* Turn off hardware LED */
	LED_turnOff(GREEN);
	floorElevator[elevatorPosition] = false;
	hValue = false;
}
/**
 * Callback funkcia pre dané komunikačné rozhranie UART.
 * Vo funkcii sa po zavolaní nastavujú príznaky, ktoré značia že komunikácia bola uspešne dokončená.
 * @param: base - adresa rozhrania ktoré vyvolalo callback funkciu
 * @param: handle - handler daného rozhrania
 * @param: status - indikuje stav po ktorom bola funkcia vyvolaná
 * @param: userData - dodatočné použivatelské dáta
 */
void uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData){
	if (kStatus_LPSCI_TxIdle == status)
	{
		txFinished = true;
	}
	if (kStatus_LPSCI_RxIdle == status)
	{
		rxFinished = true;
	}
}

/**
 * Funkcia predstavuje prvé vlákno v ktorom je zabezpečené prijímanie dát z výťahu.
 * Vo funkcii sa nestále vykonáva čitanie dat. Po prečítaní celej správy sa vyvolá následná obsluha tlačidiel alebo snímačov.
 * @param: pvParameters
 */
void Controler::recvTask(void* pvParameters)
{
	uint8_t msgSize;
	Message mess;

	for(;;)
	{	/* Prijatie prveho bytu 0xA0 */
		receiver.dataSize = 1;
		receiver.data = mess.data;
		do{
			rxFinished = false;
			LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle ,(lpsci_transfer_t*) &receiver,(size_t*) &size);
			while(!rxFinished){
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
		}while(mess.data[0] != 0xA0);

		/* Prijatie [SenderAddress][ReceiverAddress][SizeOfData][FirstByteData] */
		receiver.dataSize = 4;
		receiver.data = mess.data+1;
		rxFinished = false;
		LPSCI_TransferReceiveNonBlocking(UART0, (lpsci_handle_t*) &uart_handle ,(lpsci_transfer_t*) &receiver,(size_t*) &size);
		while(!rxFinished){
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

		/* Kontrola ci sa jedna o ACK alebo klasicku spravu */
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
		}
		else
		{
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

/**
 * Funkcia predstavuje druhé vlákno, v ktorom sa vykonáva riadenie motora.
 * Vo funkcii sa prachádza podmienkami a v prípade potreby sa spúšťa motor.
 */


void Controler::motorControlTask(void* pvParameters)
{
//	uint8_t counter = 0;

	vTaskDelay(100 / portTICK_PERIOD_MS);
	initElevator();
	PRINTF("After INIT\n");
	bool isNextFloor = false;
	hValue = false;
	for(;;)
	{
		/** V pripade ak niektory snimac nezosnima polohu vytahu nastanie blokovanie, po naplneni countera ca pohyb vykonava dalej; */
//		if(counter > 20)
//		{
//			elevatorIsMoving = false;
//			counter = 0;
//		}
		if( !elevatorIsMoving && (floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
		{
			if(elevatorDirection == UP){
				if(isNextFloor || hValue){
					/* Door OPEN */
					openDoor(elevatorPosition);
				}

				/* Next moving of elevator */
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

				}
				else
				{
					elevatorDirection = STOP;
					isNextFloor =false;
				}
			}
			else if (elevatorDirection == DOWN)
			{
				if(isNextFloor || hValue){
					/* Door OPEN*/
					openDoor(elevatorPosition);
				}

				/* Next moving of elevator */
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
//			if((floorElevator[0] || floorElevator[1] || floorElevator[2] ||floorElevator[3] ||floorElevator[4]))
//			{
//				counter++;
//			}
		}
	}
}

/**
 * Obsluha prerušenia na Porte A, ktoré je vyvolávane akcelerometrom po narušení dosky dotykom.
 * V rutine prebieha čítanie status registra akceleromtra, vyčistenie priznaku daného prerušenia a aktivovanie bezpečnostnej brzdy.
 */
void PORTA_DriverIRQHandler(void){

	uint8_t retStatus;
	accelerometer->readRegs(0x0C, &retStatus, 1);
	if(retStatus == 0x04)
	{
		accelerometer->readRegs(0x16, &retStatus, 1);
	}
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT2_PIN);
	PORT_ClearPinsInterruptFlags(PORTA, BOARD_INITPINS_ACCEL_INT1_PIN);

	if(retStatus > 0) {
		communicator->emergencyBreak(true);
		LED_switch(RED);
	}
}

/**
 * Obsluha prerušenia na PIT periferii, ktoré je vyvolávane PIT časovačom
 * Obsluha slúži na komunikáciu s WatchDog systémom, ktorý informuje o funkčnosti systému.
 */
void PIT_IRQHandler(void)
{
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	communicator->watchDogHandler();
	LED_switch(BLUE);
}

