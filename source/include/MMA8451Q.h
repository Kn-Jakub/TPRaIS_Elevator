/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MMA8451Q_H
#define MMA8451Q_H

#include <stdint-gcc.h>
#include "I2C.h"


class MMA8451Q
{
public:
  /**
  * MMA8451Q constructor
  *
  * @param addr addr of the I2C peripheral
  */
  MMA8451Q(int addr);

  /**
  * MMA8451Q destructor
  */
  ~MMA8451Q();

  /**
   * Get the value of the WHO_AM_I register
   *
   * @returns WHO_AM_I value
   */
  uint8_t getWhoAmI();

  /**
   *
   */
  void init();

  void freefall();

  /**
   * Get X axis acceleration
   *
   * @returns X axis acceleration
   */
  float getX();

  /**
   * Get Y axis acceleration
   *
   * @returns Y axis acceleration
   */
  float getY();

  /**
   * Get Z axis acceleration
   *
   * @returns Z axis acceleration
   */
  float getZ();

  /**
   * Get XYZ axis acceleration
   *
   * @param res array where acceleration data will be stored
   */
  void getAllAxis(float * res);

  uint8_t readRegs(uint8_t regg, uint8_t* data, uint8_t size);



private:
  int m_addr;
  I2C m_I2C;
  int16_t getAxis(uint8_t addr);
};

#endif
