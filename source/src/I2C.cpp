/**
 * @file    I2C.cpp
 * @author	Jakub Pekar
 * @brief   Súbor obsahújúci zdrojové kódy I2C objektu pre komunikáciu po I2C zbernici
 * @date	5 .10. 2018
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

/**
 * Konštruktor objektu I2C, ktorý zaobaľuje komunikáciu s akcelerometrom prostredníctvom rozhrania I2C
 * @param clockRate	-časovnia hodín potrebné pre komunikáciu
 */
I2C::I2C(uint32_t clockRate):
m_MasterTransfer(),
m_masterConfig()
{
	I2C_MasterGetDefaultConfig(&m_masterConfig);
	I2C_MasterInit(I2C0, &m_masterConfig, clockRate);
}

/**
 * Metóda pre odosielanie správ. Nadefinuje potrebné hodnoty pre odoslanie správy na I2C zbernicu
 * @param device_addr	-adresa zariadenia
 * @param reg_addr		-adresa registra ktorému je správa adresovaná
 * @param value			-smerník na správu
 * @param size			-veľkosť spravy
 * @param flags			-príznak rozoznávajúci metódu komunikácie
 */
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
	return 0;
}

/**
 * Metóda pre prijímanie správ. Nadefinuje potrebné hodnoty pre prijímanie správy z I2C zbernice. Využíva blokovací režim prijímania.
 * @param device_addr	-adresa zariadenia
 * @param reg_addr		-adresa registra ktorému je správa adresovaná
 * @param buff			-smerník na správu
 * @param size			-veľkosť spravy
 * @param flags			-príznak rozoznávajúci metódu komunikácie
 * @returns počet prečítaných znakov
 */

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






