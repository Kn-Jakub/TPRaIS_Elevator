/*
 * Communicator.h
 *
 *  Created on: 3. 12. 2018
 *      Author: Jakub Pekar
 */

/*
 * 		1B				 1B					1B           1B
 * |  StartByte  | ReceiverAddress  | SenderDdress  | DataSize | ... DATA  ... | CRC |
 *
 *
 */

#ifndef SRC_COMMUNICATOR_H_
#define SRC_COMMUNICATOR_H_
#include <include/RingBuffer.h>
#include <stdint-gcc.h>
#include <fsl_lpsci.h>

#define PACKET_SIZE 	261
#define LED_OUT_P 	0x10
#define LED_OUT_1   0x11
#define LED_OUT_2   0x12
#define LED_OUT_3   0x13
#define LED_OUT_4   0x14

#define LED_IN_P 	0x20
#define LED_IN_1   	0x21
#define LED_IN_2   	0x22
#define LED_IN_3   	0x23
#define LED_IN_4   	0x24

#define DISPLAY		0x30
#define MOTOR		0xf1

#define CABIN		0xF0

#define TERMINAL  	0xd0
#define WATCHDOG	0xfe
#define EMERGENCY_BREAK	0x0f

#if defined(__cplusplus)
extern "C" {
#endif
	//void UART0_IRQHandler(void);
	static volatile uint8_t sendBuffer[PACKET_SIZE];
	static volatile uint8_t rxChar;
	static volatile size_t size = 1;

	static volatile lpsci_transfer_t sender;
	static volatile lpsci_transfer_t receiver;

	static volatile lpsci_handle_t uart_handle;

	static volatile bool txFinished = true;
	static volatile bool rxFinished = true;

#if defined(__cplusplus)
} //extern C
#endif

struct Message{
	uint8_t head[3];
	uint8_t data[10];

	uint8_t getSize()
	{
		return head[2];
	}

	uint8_t getSenderAddress()
	{
		return head[1];
	}

	uint8_t getReceiverAddress()
	{
		return head[0];
	}

	uint8_t getCRC()
	{
		return data[this->getSize()+1];
	}
};

enum Direction{
	UP = 0,
	DOWN = 1,
	STOP

};

static uint8_t CRCTab[] =
	    {0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

static RingBuffer* global_buffer;

class Communicator {
public:
	Communicator(RingBuffer* buffer);
	virtual ~Communicator();

	bool sendCommand(uint8_t elementAddress, uint8_t* data, uint8_t dataSize);
	bool receivingData(Message& message);
	void setLed(uint8_t ledAddress, bool state);

	void emergencyBreak(bool ON);
	void controlDisplay(uint8_t floor,Direction direction);
	void cabineLock(bool lock);
	void controlMotor(uint8_t speed, Direction dir);
	void watchDogHandler();
	void writeToConsole(uint8_t* message,uint8_t size);
	bool verifyMessage(Message& message);
	uint8_t calcCRC(uint8_t receiverAddress, uint8_t senderAddress, uint8_t* data, uint8_t dataSize);
	static void uart_callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData);
private:
	void UARTInit();

private:
	RingBuffer* _buffer;
	uint8_t _myAddress;

};

#endif /* SRC_COMMUNICATOR_H_ */
