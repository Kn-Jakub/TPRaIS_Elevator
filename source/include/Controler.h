/*
 * Controler.h
 *
 *  Created on: 22. 12. 2018
 *      Author: jakub
 */

#ifndef CONTROLER_H_
#define CONTROLER_H_


#include "include/MMA8451Q.h"
//#include <include/Timer.h>
#include "include/LED.h"
#include <fsl_lpsci.h>

//Kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#if defined(__cplusplus)
extern "C" {
#endif

	void PIT_IRQHandler();
	void PORTA_DriverIRQHandler(void);


} //extern C

class Controler{
public:
	Controler();
	~Controler() = default;
	bool Run();
	static void recvTask(void* pvParameters);
	static void controlTask1(void* pvParameters);
	static void uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData);
private:
};

#endif /* CONTROLER_H_ */
