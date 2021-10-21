/*
    stm32f4xx_rng.c - RNG block for STM32F4xx

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

#ifndef STM32F4XX_RNG_H
#define STM32F4XX_RNG_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "stm32.h"
#include "sysemu/block-backend.h"
#include "../utility/macros.h"

#define R_RNG_MAX (0x0C/4U)

#define TYPE_STM32F4XX_RNG "stm32f4xx-rng"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f4xxRNGState, STM32F4XX_RNG);

typedef struct Stm32f4xxRNGState {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    union 
    {
        uint32_t raw[R_RNG_MAX];
        struct 
        {
            struct {
                uint32_t _unused :2;
                REG_B32(RNGEN);
                REG_B32(IE);
                uint32_t :28;
            } QEMU_PACKED CR;
            struct {
                REG_B32(DRDY);
                REG_B32(CECS);
                REG_B32(SECS);
                uint32_t _unused :2;
                REG_B32(CEIS);
                REG_B32(SEIS);
                uint32_t :25;
            } QEMU_PACKED SR;
            uint32_t DR;
        } defs;
    } regs;

    Stm32Rcc *rcc; // RCC for clock speed. 

    qemu_irq irq;

    int64_t next_drdy;

} Stm32f4xxRNGState;

#endif //STM32F4XX_RNG_H
