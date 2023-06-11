/*
    - DMA register data for:
	- STM32F03x
	- STM32G07x

	Copyright 2023 VintagePC <https://github.com/vintagepc/>

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

#ifndef HW_ARM_STM32_DMA_REGDEF_H
#define HW_ARM_STM32_DMA_REGDEF_H

enum reg_index {
	RI_ISR,
	RI_IFCR,
	RI_CHAN_BASE,
	RI_CHAN1 = RI_CHAN_BASE,
	RI_CHAN2 = RI_CHAN1 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN3 = RI_CHAN2 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN4 = RI_CHAN3 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN5 = RI_CHAN4 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN6 = RI_CHAN5 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN7 = RI_CHAN6 + STM32_COM_DMA_CHAN_REGS,
	RI_CHAN_END = RI_CHAN_BASE + (STM32_COM_DMA_CHAN_REGS * STM32_COM_DMA_MAX_CHAN),
	RI_END = RI_CHAN_END,
};

enum channel_offset
{
	CH_OFF_CCR,
	CH_OFF_CNDTR,
	CH_OFF_CPAR,
	CH_OFF_CMAR,
	CH_OFF_END,
};

#endif
