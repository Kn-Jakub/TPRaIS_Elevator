/**
 * @file    MMA8451Q.h
 * @author	Jakub Pekar
 * @brief   Súbor obsahújúci deklarácie objektu pre konfiguráciu a komunikáciu s Akcelerometrom MMA8415Q na vývojovej doske FRMD-KL25Z
 * @date 	5. 10. 2018
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
  ~MMA8451Q() = default;

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

  void tapDetection();

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

  uint8_t readRegs(uint8_t reg, uint8_t* data, uint8_t size);

private:
  int m_addr;
  I2C m_I2C;
  int16_t getAxis(uint8_t addr);
};

#endif
