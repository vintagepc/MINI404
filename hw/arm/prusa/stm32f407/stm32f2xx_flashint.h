/*
    stm32f2xx_flashint.h - Flash I/F Configuration block for STM32

	Copyright 2021-2022 VintagePC <https://github.com/vintagepc/>

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
#include "qemu-common.h"
#include "../stm32_common/stm32_common.h"

#define STM32_FINT_MAX     (0x18 / 4)

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XX_STRUCT_NAME(FlashIF), STM32F4xx_FINT)

REGDEF_BLOCK_BEGIN()
    REG_K32(LATENCY, 3);
    REG_R(5);
    REG_B32(PRFTEN);
    REG_B32(ICEN);
    REG_B32(DCEN);
    REG_B32(ICRST);
    REG_B32(DCRST);
REGDEF_BLOCK_END(flashif, acr);

REGDEF_BLOCK_BEGIN()
    REG_B32(EOP);
    REG_B32(OPER);
    REG_R(2);
    REG_B32(WRPERR);
    REG_B32(PGAERR);
    REG_B32(PGPERR);
    REG_B32(PGSERR);
    REG_R(8);
    REG_B32(BSY);
    REG_R(15);
REGDEF_BLOCK_END(flashif, sr);

REGDEF_BLOCK_BEGIN()
    REG_B32(PG);
    REG_B32(SER);
    REG_B32(MER);
    REG_K32(SNB, 4);
    REG_RB();
    REG_K32(PSIZE,2);
    REG_R(6);
    REG_B32(STRT);
    REG_R(7);
    REG_B32(EOPIE);
    REG_R(6);
    REG_B32(LOCK);
REGDEF_BLOCK_END(flashif, cr);

REGDEF_BLOCK_BEGIN()
    REG_B32(OPTLOCK);
    REG_B32(OPTSTRT);
    REG_K32(BOR_LEV,2);
    REG_RB();
    REG_B32(WDG_SW);
    REG_B32(nRST_STOP);
    REG_B32(nRST_STDBY);
    REG_K32(RDP,8);
    REG_K32(nWRP, 12);
    REG_R(4);
REGDEF_BLOCK_END(flashif, optcr);


typedef struct STM32F4XX_STRUCT_NAME(FlashIF) {
    STM32Peripheral parent;
    MemoryRegion iomem;

    union {
        struct {
			REGDEF_NAME(flashif, acr) ACR;
            uint32_t KEYR;
            uint32_t OPTKEYR; 
            REGDEF_NAME(flashif, sr) SR;
            REGDEF_NAME(flashif, cr) CR;
            REGDEF_NAME(flashif, optcr) OPTCR;
		} defs;
        uint32_t raw[STM32_FINT_MAX];
    } regs;


    uint8_t flash_state;

    MemoryRegion* flash;

    qemu_irq irq;

} STM32F4XX_STRUCT_NAME(FlashIF);

#endif //#ifndef STM32F2XX_FINT_H
