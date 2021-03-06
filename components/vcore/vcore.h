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

#ifndef _RASPBERRY_VCORE_H_
#define _RASPBERRY_VCORE_H_

#include <rabbits/component/slave.h>
#include <rabbits/component/master.h>
#include <rabbits/component/port/out.h>

#include "mailbox.h"
#include "fb.h"

class rpi_vcore: public Master<>
{
private:
    vcore_mbox m_mbox;
    rpi_vcore_fb m_fb;

public:
    OutPort<bool> p_mailbox_irq;
    TlmTargetPort<> p_mailbox_mem;

    SC_HAS_PROCESS (rpi_vcore);
    rpi_vcore(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);

    virtual ~rpi_vcore() {}

    vcore_mbox * get_mbox() { return &m_mbox; }
    rpi_vcore_fb * get_fb() { return &m_fb; }

    virtual void bus_read(uint64_t addr, uint8_t *data, unsigned int len);
    virtual void bus_write(uint64_t addr, uint8_t *data, unsigned int len);

    uint32_t vcore_to_arm_addr(uint32_t addr);
    uint32_t arm_to_vcore_addr(uint32_t addr);

    void end_of_elaboration() {
        AddressRange r(0, 0x1000);
        DmiInfo dmi_info;
        bool probe = p_bus.dmi_probe(r, dmi_info);
        if(probe) {
            MLOG_F(APP, DBG, "found memory backdoor, addr=%p\n", dmi_info.ptr);
            m_fb.mem_backdoor = dmi_info.ptr;
        }
    }
};

#endif
