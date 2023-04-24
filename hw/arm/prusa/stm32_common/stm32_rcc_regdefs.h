/*
    stm32_rcc_regdefs.h  - Common STM32 RCC register layouts.
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

#ifndef HW_ARM_STM32_RCC_REGDEF_H
#define HW_ARM_STM32_RCC_REGDEF_H

#include "stm32_common.h"

// This CR layout is common between the F030x and F4xx series.
REGDEF_BLOCK_BEGIN()
		REG_B32(HSION);
		REG_B32(HSIRDY);
		REG_RB();
		REG_K32(HSITRIM ,5);
		REG_K32(HSICAL  ,8);
		REG_B32(HSEON);
		REG_B32(HSERDY);
		REG_B32(HSEBYP);
		REG_B32(CSSON);
		REG_R(4);
		REG_B32(PLLON);
		REG_B32(PLLRDY);
		REG_B32(PLLI2SON);
		REG_B32(PLLI2SRDY);
		REG_R(4);
REGDEF_BLOCK_END(rcc_com,cr);

// This BDCR layout is common between the G070x, F030x and F4xx series.
REGDEF_BLOCK_BEGIN()
	REG_B32(LSEON);
	REG_B32(LSERDY);
	REG_B32(LSEBYP);
	REG_K32(LSEDRV,2);
	REG_B32(LSECSSON); // G070 only
	REG_B32(LSECCSD);  // G070 only
	REG_RB();
	REG_K32(RTC_SEL,2);
	REG_R(5);
	REG_B32(RTCEN);
	REG_B32(BDRST);
	REG_R(7);
	REG_B32(LSCOEN); // G070 only
	REG_B32(LSCOSEL); // G070 only
	REG_R(6);
REGDEF_BLOCK_END(rcc_com, bdcr)

#endif  //HW_ARM_STM32_RCC_REGDEF_H
