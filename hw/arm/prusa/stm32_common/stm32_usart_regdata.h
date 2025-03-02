/*
    stm32_syscfg.c - USART register data for:
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

#ifndef HW_ARM_STM32_USART_REGDEF_H
#define HW_ARM_STM32_USART_REGDEF_H

enum reg_index {
	RI_CR1,
	RI_CR2,
	RI_CR3,
	RI_BRR,
	RI_GTPR,
	RI_RTOR,
	RI_RQR,
	RI_ISR,
	RI_ICR,
	RI_RDR,
	RI_TDR,
	RI_PRESC,
	RI_END
};

#endif
