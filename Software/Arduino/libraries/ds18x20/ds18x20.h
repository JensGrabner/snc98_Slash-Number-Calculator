/*****************************************************************************
 
 DS18x20 library
 
 Copyright (C) 2016 Falk Brunner
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 
 You can contact the author at Falk.Brunner@gmx.de
 
*****************************************************************************/
 
/**
\mainpage

 \par ds18x20.h - DS18x20 library

 \author Falk Brunner

 \version 1.00
 
 \par License:
 \subpage LICENSE Beerware License
 
 \par Files:
	\subpage ds18x20.h \n
	\subpage ds18x20.c \n

 \par Developed on AVR plattform, but can be used with any other
      AVR-Studio 4.18, WinAVR20100110 (avr-gcc 4.3.3) \n
      Atomic access must be adapted to other plattforms!

 \par Example:
 \include ./onewire_demo.ino
 \page LICENSE Beerware License
 \include ./beerware.txt
 \page DS18X20.H ds18x20.h
 \include ./ds18x20.h
 \page DS18X20.C ds18x20.c
 \include ./ds18x20.c
*/

#ifndef DS18x20_H_
#define DS18x20_H_

#include <stdint.h>

/** \defgroup DS18x20_COMMANDS DS18x20 COMMANDS
  command codes for DS18x20
*/
/*@{*/
#define DS1820_CMD_SKIP_ROM          0xCC
#define DS1820_CMD_CONVERT_T         0x44
#define DS1820_CMD_READ_SCRATCHPAD   0xBE
#define DS1820_CMD_WRITE_SCRATCHPAD  0x4E
#define DS1820_CMD_COPY_SCRATCHPAD   0x48
#define DS1820_CMD_RECALL_E2         0xB8
#define DS1820_CMD_READ_POWER_SUPPLY 0xB4
/*@}*/

/** \defgroup DS18x20_ID DS18x20 IDs
  ID codes for DS18x20
*/
/*@{*/
#define DS18B20_ID 0x28
#define DS18S20_ID 0x10
/*@}*/

/** \defgroup DS18X20_FUNCTIONS DS18X20 FUNCTIONS 
  * standard functions
  */
/*@{*/ 

/**
 \brief start temperature conversion
 \param parasitic_power set true if device has parasitic power supply
 \return error code
 \return 0: no error
 \return 1: no presence pulse detected
 */   
uint8_t  ds18x20_convert_t(uint8_t parasitic_power);

#define ds18B20_convert_t(x) ds18x20_convert_t(x)
#define ds18S20_convert_t(x) ds18x20_convert_t(x)

/**
 \brief Read temperature from DS18x20
 \param none
 \return temperature as 16 bit signed data (plain)
 */   

int16_t ds18x20_read_temp(void);


/**
 \brief Read temperature from DS18B20 (12 bit resolution)
 \param none
 \return temperature in 1/10 C (fixed point)
 */   

int16_t ds18B20_read_temp(void);

/**
 \brief Read temperature from DS18S20 (9 bit resolution)
 \param none
 \return temperature in 1/10 C (fixed point)
 */   

int16_t ds18S20_read_temp(void);

/**
 \brief Read complete scratchpad of DS18x20 (9 bytes)
 \param *buffer pointer to data array
 \return none
 */   

void ds18x20_read_scratchpad(uint8_t *buffer);

#define ds18B20_read_scratchpad(x) ds18x20_read_scratchpad(x)
#define ds18S20_read_scratchpad(x) ds18x20_read_scratchpad(x)

/**
 \brief write th and tl of DS18S20
 \param th upper temperature limit (1 C resolution)
 \param tl lower temperature limit (1 C resolution)
 \return none
 */   

void ds18S20_write_scratchpad(int8_t th, int8_t tl);

/**
 \brief write th, tl and configuration of DS18B20
 \param th upper temperature limit (1 C resolution)
 \param tl lower temperature limit (1 C resolution)
 \param config configuration byte
 \return none
 */   

void ds18B20_write_scratchpad(uint8_t th, uint8_t tl, uint8_t config);

/**
 \brief copy scratchpad to EEPROM, busy waiting (10ms),
 \brief parasitic power switching is completely handled inside the function, no need for further user action
 \param parasitic_power set true if device has parasitic power supply
 \return none
 */   

void ds18x20_copy_scratchpad(uint8_t parasitic_power);

#define ds18B20_copy_scratchpad(x) ds18x20_copy_scratchpad(x);
#define ds18S20_copy_scratchpad(x) ds18x20_copy_scratchpad(x);

/**
 \brief copy EEPROM to scratchpad, busy waiting (1ms)
 \param none
 \return none
 */   

void ds18x20_recall_E2(void);
#define ds18B20_recall_E2(x) ds18x20_recall_E2(x)
#define ds18S20_recall_E2(x) ds18x20_recall_E2(x)

/**
 \brief read power supply
 \param none
 \return power mode
 \return 0: normal power
 \return 1: parasitic power
 */   

uint8_t ds18x20_read_power_supply(void);

#define ds18B20_read_power_supply(x) ds18x20_read_power_supply(x)
#define ds18S20_read_power_supply(x) ds18x20_read_power_supply(x)

/*@}*/

#endif
