/*
 * 		I2C.cpp
 *      Author: Jakub Pekar
 */

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "pin_mux.h"

#include "../include/I2C.h"

#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U

I2C::I2C(uint32_t clockRate):
m_MasterTransfer(),
m_masterConfig()
{
	I2C_MasterGetDefaultConfig(&m_masterConfig);
	I2C_MasterInit(I2C0, &m_masterConfig, clockRate);
}

I2C::~I2C() {}

uint8_t I2C::write(uint8_t device_addr, uint8_t reg_addr, uint8_t*  value, uint8_t size, uint32_t flags )
{

	m_MasterTransfer.slaveAddress = device_addr;
	m_MasterTransfer.direction = kI2C_Write;
	m_MasterTransfer.subaddress = reg_addr;
	if(reg_addr != 255)
	{
		m_MasterTransfer.subaddressSize = 1;
	} else {
		m_MasterTransfer.subaddress = 0;
		m_MasterTransfer.subaddressSize = 0;
	}
	m_MasterTransfer.data = value;
	m_MasterTransfer.dataSize = size;
	m_MasterTransfer.flags = flags;

	I2C_MasterTransferBlocking(I2C0, &m_MasterTransfer);

}

uint8_t I2C::read(uint8_t device_addr, uint8_t reg_addr, uint8_t*  buff, uint8_t size, uint32_t flags)
{
	m_MasterTransfer.slaveAddress = device_addr;
	m_MasterTransfer.direction = kI2C_Read;

	if(reg_addr != 255)
	{
		m_MasterTransfer.subaddress = reg_addr;
		m_MasterTransfer.subaddressSize = 1;
	} else {
		m_MasterTransfer.subaddress = 0;
		m_MasterTransfer.subaddressSize = 0;
	}
	m_MasterTransfer.data = buff;
	m_MasterTransfer.dataSize = size;
	m_MasterTransfer.flags = flags;

	I2C_MasterTransferBlocking(I2C0, &m_MasterTransfer);
	return 1;
}






