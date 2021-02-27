/*
    stm32f4xx_itm.h - ITM Debug channel for STM32

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef STM32F2XX_ITM_H
#define STM32F2XX_ITM_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"

#define R_ITM_PORT_BASE     (0x00 / 4)
#define R_ITM_TER   (0xE00 / 4)
#define R_ITM_TPR   (0xE40 / 4)
#define R_ITM_TCR   (0xE80 / 4)
#define R_ITM_LAR     (0xFB0 / 4)
#define R_ITM_MAX     (0xFFC / 4)

#define TYPE_STM32F4XX_ITM "stm32f4xx-itm"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f4xx_itm, STM32F4XX_ITM)

struct stm32f4xx_itm {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[R_ITM_MAX];
    uint32_t port_buffer[101];
    uint8_t buffer_pos;
    bool unlocked; // Set when LAR is written properly
};

#endif //#ifndef STM32F2XX_ITM_H
