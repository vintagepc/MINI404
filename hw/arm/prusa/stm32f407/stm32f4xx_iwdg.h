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
#include "../stm32_common/stm32_common.h"

#define R_IWDG_MAX 4

typedef struct Stm32Rcc Stm32Rcc;

#define TYPE_STM32F4XX_IWDG "stm32f4xx-iwdg"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f4xx_iwdg, STM32F4XX_IWDG);

typedef struct stm32f4xx_iwdg {
    STM32Peripheral parent;
    MemoryRegion  iomem;

    union {
        struct {
            struct {
                uint32_t KEY :16;
                uint32_t :16;
            } QEMU_PACKED KR;
            struct {
                uint32_t PR :3;
                uint32_t :29;
            } QEMU_PACKED PR;
            struct {
                uint32_t RL :12;
                uint32_t :20;
            } QEMU_PACKED RLR;
            struct {
                uint32_t PVU :1;
                uint32_t RVU :1;
                uint32_t :30;
            } QEMU_PACKED SR;
        } QEMU_PACKED defs;
        uint32_t all[R_IWDG_MAX];
    } QEMU_PACKED regs;

    QEMUTimer *timer;

    bool time_changed, started;

    Stm32Rcc *rcc; // RCC for clock speed. 

} stm32f4xx_iwdg;

#endif //STM32F4XX_IWDG_H
