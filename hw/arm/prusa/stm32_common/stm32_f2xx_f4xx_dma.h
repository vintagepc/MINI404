/*
    stm32_dma.h - DMA for STM32F2/4.
	So far the DMA layout is common to the following chips:
	- F4xx series,
	- F2xx series (? - it's modelled off the Pebble QEMU version which supports this.),

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


#ifndef STM32_F2xx_F4xx_DMA_H
#define STM32_F2xx_F4xx_DMA_H

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "stm32_common.h"
#include "stm32_shared.h"

#define STM32_F2xx_DMA_MAX_CHAN 8
#define STM32_F2xx_DMA_CHAN_REGS 6

#define R_DMA_MAX (4+(STM32_F2xx_DMA_MAX_CHAN*STM32_F2xx_DMA_CHAN_REGS))

OBJECT_DECLARE_SIMPLE_TYPE(STM32F2XX_STRUCT_NAME(Dma), STM32F4xx_DMA);

#define _STM32_DMA_INT_BITSET(chan) \
	REG_B32(_JOIN2R(FEIF,chan)); \
	REG_R(1); \
	REG_B32(_JOIN2R(DMEIF,chan)); \
	REG_B32(_JOIN2R(TEIF,chan)); \
	REG_B32(_JOIN2R(HTIF,chan)); \
	REG_B32(_JOIN2R(TCIF,chan));


REGDEF_BLOCK_BEGIN()
	_STM32_DMA_INT_BITSET(0)
	_STM32_DMA_INT_BITSET(1)
	REG_R(4);
	_STM32_DMA_INT_BITSET(2)
	_STM32_DMA_INT_BITSET(3)
	REG_R(4);
REGDEF_BLOCK_END(dma, low_int)

REGDEF_BLOCK_BEGIN()
	_STM32_DMA_INT_BITSET(4)
	_STM32_DMA_INT_BITSET(5)
	REG_R(4);
	_STM32_DMA_INT_BITSET(6)
	_STM32_DMA_INT_BITSET(7)
	REG_R(4);
REGDEF_BLOCK_END(dma, high_int)

#undef _STM32_DMA_INT_BITSET

REGDEF_BLOCK_BEGIN()
	REG_B32(EN);
	REG_B32(DMEIE);
	REG_B32(TEIE);
	REG_B32(HTIE);
	REG_B32(TCIE);
	REG_B32(PFCTRL);
	REG_K32(DIR,2);
	REG_B32(CIRC);
	REG_B32(PINC);
	REG_B32(MINC);
	REG_K32(PSIZE,2);
	REG_K32(MSIZE,2);
	REG_B32(PINCOS);
	REG_K32(PL,2);
	REG_B32(DBM);
	REG_B32(CT);
	REG_R(1);
	REG_K32(PBURST,2);
	REG_K32(MBURST,2);
	REG_K32(CHSEL,3);
	REG_R(4);
REGDEF_BLOCK_END(dma, sxcr)

REGDEF_BLOCK_BEGIN()
	REG_K32(FTH,2);
	REG_B32(DMDIS);
	REG_K32(FS,3);
	REG_R(1);
	REG_B32(FEIE);
	REG_R(24);
REGDEF_BLOCK_END(dma, sxfcr)

#define _STM32_DMA_CHAN_BLK(x) \
	REGDEF_NAME(dma, sxcr) _JOIN2R(SCR,x); \
	REG_S32(NDTR, 16) _JOIN2R(SNDTR,x); \
	uint32_t _JOIN2R(SPAR,x); \
	uint32_t _JOIN2R(SM0AR,x); \
	uint32_t _JOIN2R(SM1AR,x); \
	REGDEF_NAME(dma, sxfcr) _JOIN2R(SFCR,x);



typedef struct STM32F2XX_STRUCT_NAME(Dma) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

    union {
        struct {
			REGDEF_NAME(dma, low_int) LISR;			//0x00
			REGDEF_NAME(dma, high_int) HISR;		//0x04
			REGDEF_NAME(dma, low_int) LIFCR;		//0x08
			REGDEF_NAME(dma, high_int) HIFCR;		//0x0C
			_STM32_DMA_CHAN_BLK(0)	 				//0x10 - 0x24
			_STM32_DMA_CHAN_BLK(1)	 				//0x28 - 0x3C
			_STM32_DMA_CHAN_BLK(2)	 				//0x40 - 0x54
			_STM32_DMA_CHAN_BLK(3)	 				//0x58 - 0x6C
			_STM32_DMA_CHAN_BLK(4)	 				//0x70 - 0x84
			_STM32_DMA_CHAN_BLK(5)	 				//0x88 - 0x9C
			_STM32_DMA_CHAN_BLK(6)	 				//0xA0 - 0xB4
			_STM32_DMA_CHAN_BLK(7)	 				//0xB8 - 0xCC
		} defs;
        uint32_t raw[R_DMA_MAX];
    } regs;

	uint32_t original_ndtrs[STM32_F2xx_DMA_MAX_CHAN];

	qemu_irq irq[STM32_F2xx_DMA_MAX_CHAN];

} STM32F2XX_STRUCT_NAME(Dma);

#undef _STM32_DMA_CHAN_BLK

#endif //STM32_F2xx_F4xx_DMA_H
