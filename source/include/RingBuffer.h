/*
 * RingBuffer.h
 *
 *  Created on: 29. 12. 2018
 *      Author: jakub
 */

#ifndef INCLUDE_RINGBUFFER_H_
#define INCLUDE_RINGBUFFER_H_
#include <stdint-gcc.h>
#include <cstddef>

#define RING_BUFFER_SIZE 261
enum BufferState{
	FULL,
	EMPTY,
	DATA
};

class RingBuffer {
public:
	RingBuffer();
	virtual ~RingBuffer();
	bool push(uint8_t* data,size_t size);
	void getData(uint8_t* data,size_t size);
	uint8_t pop();
	uint8_t dataSize();
private:
	void setState();

private:
	uint8_t _buffer[RING_BUFFER_SIZE];
	size_t _sizeOfBuffer;
	uint8_t* _head;
	uint8_t* _tail;
	uint8_t* _endOfBuffer;
	BufferState _bufferState;
	bool _overRun;


};

#endif /* INCLUDE_RINGBUFFER_H_ */
