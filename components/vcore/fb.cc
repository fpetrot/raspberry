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

#include "fb.h"
#include "vcore.h"

#include <rabbits/ui/ui.h>

#include "rabbits/logger.h"

using namespace sc_core;

rpi_vcore_fb::rpi_vcore_fb(sc_core::sc_module_name n,
                           const Parameters &params,
                           ConfigManager &config)
    : Component(n, params, config)
{
    m_fb_state = FB_IDLE;
}

void rpi_vcore_fb::set_info(const rpi_fb_info & info)
{
    FramebufferInfo ui_info;

    LOG_F(SIM, DBG, "setting info\n");
    LOG_F(SIM, DBG, "pw: %" PRIu32 "\n", info.physical_w);
    LOG_F(SIM, DBG, "ph: %" PRIu32 "\n", info.physical_h);
    LOG_F(SIM, DBG, "vw: %" PRIu32 "\n", info.virtual_w);
    LOG_F(SIM, DBG, "vh: %" PRIu32 "\n", info.virtual_h);
    LOG_F(SIM, DBG, "bpp: %" PRIu32 "\n", info.bpp);
    LOG_F(SIM, DBG, "x offset: %" PRIu32 "\n", info.x_offset);
    LOG_F(SIM, DBG, "y offset: %" PRIu32 "\n", info.y_offset);

    m_info = info;

    ui_info.h = info.virtual_h;
    ui_info.w = info.virtual_w;
    ui_info.data = mem_backdoor;

    switch (info.bpp) {
    case 16:
        ui_info.pixel_info = FramebufferInfo::RGB_565;
        break;

    case 24:
        ui_info.pixel_info = FramebufferInfo::RGB_888;
        break;

    default:
        LOG_F(SIM, ERR, "Unable to setup framebuffer: unsupported mode\n");
        goto setup_err;
    }

    /* TODO: Switch to the framebuffer out port */
    if(m_ui_fb == NULL) {
        m_ui_fb = get_config().get_ui().create_framebuffer("raspberry framebuffer", ui_info);
    } else {
        m_ui_fb->set_info(ui_info);
    }

    m_fb_state = FB_RUNNING;
    return;

setup_err:
    return;
}

uint32_t rpi_vcore_fb::get_pitch() const
{
    if (m_fb_state == FB_IDLE) {
        return 0;
    }

    return m_info.virtual_w * (m_info.bpp >> 3);
}

uint32_t rpi_vcore_fb::get_base_addr() const
{
    return FB_BASE_ADDR;
}

uint32_t rpi_vcore_fb::get_size() const
{
    if (m_fb_state == FB_IDLE) {
        return 0;
    }

    return get_pitch() * m_info.virtual_h;
}

