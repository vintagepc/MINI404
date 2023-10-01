/*
    stm32_syscfg.c - SYSCFG register data for:
	- STM32F03x
	- STM32G07x
	- STM32F4xx.

	Copyright 2022-3 VintagePC <https://github.com/vintagepc/>

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

#ifndef HW_ARM_STM32_SYSCFG_REGDEF_H
#define HW_ARM_STM32_SYSCFG_REGDEF_H

enum reg_index {
	RI_MEMRMP_CFGR1, // Shared address between F4xx and [F/G]0
	RI_PMC,
	RI_EXTICR1,
	RI_EXTICR2,
	RI_EXTICR3,
	RI_EXTICR4,
	RI_CFGR2,
	RI_CMPCR = 0x20/4,
	RI_END
};

enum MEMMODE
{
	MEMMODE_MAIN,
	MEMMODE_SYSFLASH,
	MEMMODE_FMC_BANK1,
	MEMMODE_SRAM1,
	MEMMODE_SDRAM1
};

#endif
