/*
    stm32f4xx_iwdg.h - IWDG for STM32F4xx

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


#ifndef STM32F4XX_IWDG_H
#define STM32F4XX_IWDG_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "exec/memory.h"

#define R_IWDG_MAX 4

typedef struct Stm32Rcc Stm32Rcc;

#define TYPE_STM32F4XX_IWDG "stm32f4xx-iwdg"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f4xx_iwdg, STM32F4XX_IWDG);

typedef struct stm32f4xx_iwdg {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    union {
        uint32_t all[R_IWDG_MAX];
        struct {
            union {
                uint32_t KEY :16;
                uint32_t :16;
            }KR QEMU_PACKED;
            union {
                uint32_t PR :3;
                uint32_t :29;
            }PR QEMU_PACKED;
            union {
                uint32_t RL :12;
                uint32_t :20;
            }RLR QEMU_PACKED;
            union {
                uint32_t PVU :1;
                uint32_t RVU :1;
                uint32_t :30;
            }SR QEMU_PACKED;
        } defs;
    } regs QEMU_PACKED;

    QEMUTimer *timer;

    bool time_changed, started;

    Stm32Rcc *rcc; // RCC for clock speed. 

} stm32f4xx_iwdg;

#endif //STM32F4XX_IWDG_H
