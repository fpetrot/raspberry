/*
 *  This file is part of Rabbits
 *  Copyright (C) 2015  Clement Deschamps and Luc Michel
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _RASPBERRY_GPIO_H_
#define _RASPBERRY_GPIO_H_

#include "rabbits/component/slave.h"
#include <rabbits/component/port/out.h>
#include <rabbits/component/port/inout.h>
#include <rabbits/component/port/vector.h>

#define GPFSEL0 0x0
#define GPFSEL1 0x4
#define GPFSEL2 0x8
#define GPFSEL3 0xC
#define GPFSEL4 0x10
#define GPFSEL5 0x14
#define GPSET0  0x1C
#define GPSET1  0x20
#define GPCLR0 0x28
#define GPCLR1 0x2C
#define GPLEV0 0x34
#define GPLEV1 0x38
#define GPEDS0 0x40
#define GPEDS1 0x44
#define GPREN0 0x4C
#define GPREN1 0x50
#define GPFEN0 0x58
#define GPFEN1 0x5C
#define GPHEN0 0x64
#define GPHEN1 0x68
#define GPLEN0 0x70
#define GPLEN1 0x74
#define GPAREN0 0x7C
#define GPAREN1 0x80
#define GPAFEN0 0x88
#define GPAFEN1 0x8C
#define GPPUD 0x94
#define GPPUDCLK0 0x98
#define GPPUDCLK1 0x9C

const int RPI_GPIO_COUNT = 54;

class GPFSELn {
private:
    uint8_t m_fsel[RPI_GPIO_COUNT];

public:
    void reset() {
        for(int i=0; i<6; i++) {
            set(i, 0);
        }
    }

    void set(uint8_t reg, uint32_t value) {
        for(int i=0; i<10; i++) {
            uint32_t index = 10*reg + i;
            if(index < sizeof(m_fsel)) {
                int fsel = (value >> (3 * i)) & 0x3;
                m_fsel[index] = fsel;
            }
        }
    }

    uint32_t get(uint8_t reg) {
        uint32_t value = 0;
        for(int i=0; i<10; i++) {
            uint32_t index = 10*reg + i;
            if(index < sizeof(m_fsel)) {
                value |= m_fsel[index] << (3 * i);
            }
        }
        return value;
    }

    uint8_t get_function(int index) {
        if(index >= 0 && index < 54) {
            return m_fsel[index];
        }
        return false;
    }

    bool is_in(int index) {
        return get_function(index) == 0;
    }

    bool is_out(int index) {
        return get_function(index) == 1;
    }
};

class raspberry_gpio: public Slave<>
{
public:
    SC_HAS_PROCESS (raspberry_gpio);
    raspberry_gpio(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
    virtual ~raspberry_gpio();

    VectorPort< InOutPort<bool> > p_gpios;

    uint32_t m_lev0, m_lev1;

    sc_core::sc_event_or_list m_ev_gpios;

    void gpset(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev);
    void gpclr(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev);

private:
    void bus_cb_read_32(uint64_t addr, uint32_t *data, bool &bErr);
    void bus_cb_write_32(uint64_t addr, uint32_t *data, bool &bErr);

    void gpio_thread();

    void end_of_elaboration();

    GPFSELn m_gpfsel;
};

#endif
