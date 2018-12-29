/*
 * Controler.h
 *
 *  Created on: 22. 12. 2018
 *      Author: jakub
 */

#ifndef CONTROLER_H_
#define CONTROLER_H_

#include <include/Communicator.h>
#include <include/MMA8451Q.h>
#include <include/Protothread.h>
#include <include/Timer.h>
#include <include/LED.h>

#if defined(__cplusplus)
extern "C" {
#endif
	static volatile int flagIRQ = 0;

	void PIT_IRQHandler();
	void PORTA_DriverIRQHandler(void);


} //extern C


static Communicator* global_communicator;
static MMA8451Q* accelerometer;

class Controler: public Protothread {
public:
	Controler();
	~Controler() = default;
	bool Run() override;

private:

	Timer _WDTimer;
};

#endif /* CONTROLER_H_ */
