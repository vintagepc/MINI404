/*
    stm32_rcc.h  - Common STM32 RCC parent
	Copyright 2022 VintagePC <https://github.com/vintagepc/>

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

#ifndef STM32_COM_RCC_H
#define STM32_COM_RCC_H


#include "qemu/osdep.h"
#include "qom/object.h"
#include "stm32_types.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "stm32_clk.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Rcc), COM_CLASS_NAME(Rcc), STM32COM_RCC);

typedef struct COM_CLASS_NAME(Rcc) {
    SysBusDeviceClass parent;
} COM_CLASS_NAME(Rcc);

typedef struct COM_STRUCT_NAME(Rcc) {
	SysBusDevice parent;
	// Note this is not a strictly isolated parent class.
	// Array of peripheral clocks
	Clk_t pclocks[STM32_P_COUNT];
	// RCC reset vectors
	qemu_irq reset[STM32_P_COUNT];
	// IO region. Not initialized, that's up to the child.
    MemoryRegion iomem;
	// System NVIC
	qemu_irq irq;
	// The four standard clocks all systems have.
	Clk_t HSICLK, HSECLK, LSECLK, LSICLK;
	// ... and their four input frequencies:
    uint32_t hse_freq, lse_freq, hsi_freq, lsi_freq;

	DeviceRealize realize_func;

} COM_STRUCT_NAME(Rcc);


// Init a peripheral clock with auto-stringify the STM32_P enum entry
#define INIT_PCLK(periph, ... ) clktree_create_clk(&s->parent.pclocks[STM32_P_##periph], #periph, __VA_ARGS__)

// Init periph clock with scale 1, no max freq
#define INIT_PCLK_NSM(periph, sel_input, ... ) clktree_create_clk(&s->parent.pclocks[STM32_P_##periph], #periph, 1, 1, false, CLKTREE_NO_MAX_FREQ, sel_input, __VA_ARGS__)


#endif // STM32_COM_RCC_H
