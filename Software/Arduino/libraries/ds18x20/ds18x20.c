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
 
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "onewire.h"
#include "ds18x20.h"

uint8_t ds18x20_convert_t(uint8_t parasitic_power)   {

    if (onewire_reset()) {
        return 1;                               // no response
    } else {
        onewire_write_byte(DS1820_CMD_SKIP_ROM);
        if (parasitic_power) {
            //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            uint8_t sreg_tmp=SREG; cli(); // Arduino workaround :-0
                onewire_write_byte(DS1820_CMD_CONVERT_T);
                ONEWIRE_STRONG_PU_ON
            //}
            SREG = sreg_tmp;        // Arduino workaround :-0         
        } else {
            onewire_write_byte(DS1820_CMD_CONVERT_T);       
        }
     }
    return 0;
}

int16_t ds18x20_read_temp(void) {
    uint8_t lsb, msb;
    int16_t temp;

    onewire_write_byte(DS1820_CMD_READ_SCRATCHPAD);
    lsb = onewire_read_byte();
    msb = onewire_read_byte();
    onewire_reset();                    // terminate read process

    temp = ((int16_t)msb << 8) | lsb;

    return temp;
}

int16_t ds18B20_read_temp(void) {
    int16_t temp;

    temp = ds18x20_read_temp();

    // calculate temperature with 0,1 C resolution
    //
    // t(0.1C)  = t(1/16C) * 0.625
    //          = t(1/16C) * 625 / 1000
    //          = t(1/16C) * 625 * 1.024 / 1024
    //          = t(1/16C) * 640 / 1024 

    temp = (temp*640L) >> 10;   
    return temp;
}

int16_t ds18S20_read_temp(void) {
    int16_t temp;

    temp = ds18x20_read_temp();

    // calculate temperature with 0,1 C resolution
    //
    // t(0.1C)  = t(1/2C) * 5

    temp = temp*5;  
    return temp;
}

void ds18x20_read_scratchpad(uint8_t *buffer) {
    uint8_t i;

    onewire_write_byte(DS1820_CMD_READ_SCRATCHPAD);
    for (i=0; i<9; i++) {
        buffer[i]=onewire_read_byte();
    }
}

void ds18S20_write_scratchpad(int8_t th, int8_t tl) {

    onewire_write_byte(DS1820_CMD_WRITE_SCRATCHPAD);
    onewire_write_byte(th);
    onewire_write_byte(tl);
}

void ds18B20_write_scratchpad(uint8_t th, uint8_t tl, uint8_t config) {

    onewire_write_byte(DS1820_CMD_WRITE_SCRATCHPAD);
    onewire_write_byte(th);
    onewire_write_byte(tl);
    onewire_write_byte(config);
}

void ds18x20_copy_scratchpad(uint8_t parasitic_power) {

    if (parasitic_power) {
        //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        uint8_t sreg_tmp=SREG; cli(); // Arduino workaround :-0
            onewire_write_byte(DS1820_CMD_COPY_SCRATCHPAD);
            ONEWIRE_STRONG_PU_ON
        //}
        SREG = sreg_tmp;        // Arduino workaround :-0
    } else {
        onewire_write_byte(DS1820_CMD_COPY_SCRATCHPAD);
    }
    _delay_ms(10);
    ONEWIRE_STRONG_PU_OFF
}

void ds18x20_recall_E2(void) {
    onewire_write_byte(DS1820_CMD_RECALL_E2);
    _delay_ms(1);
}

uint8_t ds18x20_read_power_supply(void) {
    onewire_write_byte(DS1820_CMD_READ_POWER_SUPPLY);
    return !onewire_read_bit();
}
