/*
 * I2C.h
 *
 *  Created on: 5. 11. 2018
 *      Author: Jakub Pekar
 */

#ifndef I2C_H_
#define I2C_H_

#include "fsl_i2c.h"

class I2C {
public:
	I2C(uint32_t clockRate);
	~I2C();
	uint8_t write(uint8_t device_adrr, uint8_t reg_addr, uint8_t* value, uint8_t size, uint32_t flags = kI2C_TransferDefaultFlag);
	uint8_t read(uint8_t device_adrr, uint8_t reg_addr, uint8_t*  buff, uint8_t size, uint32_t flags = kI2C_TransferDefaultFlag);
private:
	i2c_master_transfer_t m_MasterTransfer;
	i2c_master_config_t m_masterConfig;
};

#endif /* I2C_H_ */
