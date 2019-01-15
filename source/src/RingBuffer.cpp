/*
 * RingBuffer.cpp
 *
 *  Created on: 29. 12. 2018
 *      Author: jakub
 */

#include "../include/RingBuffer.h"
#include <cassert>
#include <cstring>

RingBuffer::RingBuffer()
{
	_head = _buffer;
	_sizeOfBuffer = RING_BUFFER_SIZE;
	_tail = _buffer;
	_endOfBuffer = _buffer + _sizeOfBuffer -1;

	_bufferState = EMPTY;
	_overRun = false;

}

RingBuffer::~RingBuffer() {

}

bool RingBuffer::push(uint8_t* data,size_t size)
{
	assert((size <= _sizeOfBuffer - dataSize()) && "Write count is bigger than free capacity of Buffer");

	assert((data != NULL) && "Pointer data_pt is NULL");
	int index = 0;
	if(_sizeOfBuffer - dataSize() >= size)
	{
		if((_endOfBuffer - _head + 1) >= size){
			//memcpy(_head, data, size);
			for(int i = 0; i < size; i++)
			{
				_head[i] = data[i];
			}
			if(_head + size > _endOfBuffer)
			{
				_head = _buffer;
				_overRun = true;
			} else
			{
				_head += size;
			}
		}
		else
		{
			uint16_t pomSize = _endOfBuffer - _head + 1;
			memcpy(_head,data,pomSize);
			memcpy(_buffer,data + pomSize, size - pomSize);
			_head = _buffer + (size - pomSize);
			_overRun = true;
		}
		setState();
		return true;
	}
	else
	{
		return false;
	}
}
void RingBuffer::getData(uint8_t* data,size_t size)
{

}
uint8_t RingBuffer::pop()
{
	uint8_t pomData;
	if(_bufferState == DATA || _bufferState == FULL)
	{
		pomData = *_tail;
		if(_tail < _endOfBuffer){
			++_tail;
		}else{
			_tail = _buffer;
			_overRun = false;
		}
		setState();
		return pomData;
	} else {

		return 0;
	}
}
uint8_t RingBuffer::dataSize()
{
	if(_head >_tail){
		return _head - _tail ;
	}
	else if (_head == _tail)
	{
		if(_overRun){
			return _sizeOfBuffer;
		} else {
			return 0;
		}
	}
	else
	{
		return _sizeOfBuffer - (_tail - _head);
	}
}

void RingBuffer::setState()
{
	if(_sizeOfBuffer == dataSize())
	{
		_bufferState = FULL;
	}
	else if( dataSize() == 0)
	{
		_bufferState = EMPTY;
	}
	else
	{
		_bufferState = DATA;
	}

}
