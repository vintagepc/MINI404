/*
    stm32f2xx_flashint.h - Flash I/F Configuration block for STM32

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



#ifndef STM32F2XX_FINT_H
#define STM32F2XX_FINT_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"

#define STM32_FINT_ACR     (0x00 / 4)
#define STM32_FINT_KEYR    (0x04 / 4)
#define STM32_FINT_OPTKEYR (0x08 / 4)
#define STM32_FINT_SR      (0x0c / 4)
#define STM32_FINT_CR     (0x10 / 4)
#define STM32_FINT_OPTCR     (0x14 / 4)
#define STM32_FINT_MAX     (0x18 / 4)

#define TYPE_STM32F2XX_FINT "stm32f2xx-fint"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f2xx_fint, STM32F2XX_FINT)

struct stm32f2xx_fint {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[STM32_FINT_MAX];

	qemu_irq irq;
};

#endif //#ifndef STM32F2XX_FINT_H
