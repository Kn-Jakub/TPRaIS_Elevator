/*
 * @file    Controler.h
 * @author	Jakub Pekar
 * @brief   Súbor obsahujúci deklaráciu triedy Controler
 * @date 	14. 12. 2018
 */

#ifndef CONTROLER_H_
#define CONTROLER_H_


#include "include/MMA8451Q.h"
#include "include/LED.h"
#include <fsl_lpsci.h>

//Kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "include/definitions.h"
#include "include/Timer.h"


#if defined(__cplusplus)
extern "C" {
#endif

	void PIT_IRQHandler();
	void PORTA_DriverIRQHandler(void);

	/**
	 * Callback funkcia pre dané komunikačné rozhranie UART.
	 * Vo funkcii sa po zavolaní nastavujú príznaky, ktoré značia že komunikácia bola uspešne dokončená.
	 * @param 	base - adresa rozhrania ktoré vyvolalo callback funkciu
	 * @param 	handle - handler daného rozhrania
	 * @param 	status - indikuje stav po ktorom bola funkcia vyvolaná
	 * @param 	userData - dodatočné použivatelské dáta
	 */
	void uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData);

} //extern C

class Controler{
public:
	/**
	 * Konštruktor triedy Controler.
	 * Inicializuje atribut _timer, ktorý slúži pre odosielanie správ WatchDog modulu výťahu.
	 * Vytvára vlákna v ktorých sa vykonáva obsluha výťahu.
	 * Konfiguruje komunikačné rozhranie UART prostredníctvom LPSCI ovládača.
	 * Vytvára globálne objekty: 	- communicator - pre komunikáciu s výťahom
	 * 								- acceleromter - pre detekciu voľného pádu/detekciu dotyku
	 *
	 **/
	Controler();

	/**
	 * Deštruktor objektu Controler, v ktorom sú mazané globalné premenné communicator a accelerometer vytvárané dynamicky.
	 */
	~Controler();
	/**
	 * Funkcia spúšťa timer odosielania správ WatchDog systému, konfiguruje accelerometer na detekciu dotyku (tapDetection) a spúšťa obsluhu definovaných vlákien.
	 */
	bool Run();

	/**
	 * Funkcia predstavuje prvé vlákno v ktorom je zabezpečené prijímanie dát z výťahu.
	 * Vo funkcii sa nestále vykonáva čitanie dat. Po prečítaní celej správy sa vyvolá následná obsluha tlačidiel alebo snímačov.
	 * @param: pvParameters
	 */
	static void recvTask(void* pvParameters);

	/**
	 * Funkcia predstavuje druhé vlákno, v ktorom sa vykonáva riadenie motora.
	 * Vo funkcii sa prachádza podmienkami a v prípade potreby sa spúšťa motor.
	 */
	static void motorControlTask(void* pvParameters);

	/**
	 * Obsluha tlačidiel. Funkcia je vyvolaná po stlačení tlačidla. Na základe získanej adresy sa vykoná sekvencia príkazov,
	 * ktorá obsahuje zapnutie príslušných LED diód označujúcich poschodia na ktorých výťah bude stáť. Taktiež sa nastavia globálne hodnoty na základe ktorých prebieha riadenie motora.
	 * @param: address - adresa prvku výťahu.
	 */
	static void servedButton(uint8_t address);

	/**
	 * Obsluha sensorov výťahu. Sensory detekujú na ktorom poschodí sa aktuálne výťah nachádza.
	 * Na základe prijatej adressy sa vykoná obsluha pre daný senzor. Ak je v globálnej premennej nastavené dane poschodie ako poschodie na ktorom jhe potrebné zastaviť, odošle sa správa pre motor výťahu s príkazom zastaviť (STOP).
	 * Následne sa aktualizuje displej výťahu.
	 * @param: mess - správa z daného senzoru, ktorá obsahuje aj adresu senzora.
	 */
	static void servedLimitSensor(Message& mess);

	/**
	 * Funkcia obsluhy otvorenia dverí po zastavení na niektorom poschodí.
	 * Vo funkcii sa nastavujú LED diódy pre dané poschodie. otvoria sa dvere, počka sa určitý čas a dvere sa opäť zatvoria.
	 */
	static void openDoor(uint8_t elePosition);
	/**
	 * Inicializácia výťahu.
	 * Vo funkcii sa nadviaže komunikácia so simulátorom výťahu. Odošle sa príkaz pre zistenie aktuálnej pozícii výťahu prostredníctvom motorového enkódera, vyresetuje sa Watchdog a odistí núdzová brzda.
	 * Následne sa čaká na prijatie správy z enkódera o polohe výťahu.
	 * Na základe zistenej hodnoty sa vyhodnotí operácia s výťahom, v prípade potreby zíde výťah na prízemie.
	 * Funkcia vypne všetky LED diody výťahu.
	 */
	static void initElevator();

private:
	Timer _timer;

};

#endif /* CONTROLER_H_ */
