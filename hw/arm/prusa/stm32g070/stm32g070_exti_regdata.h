/*
    stm32g070_exti_regdata.h - Exti register data for:
	- STM32G07x

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

#ifndef HW_ARM_STM32G070_EXTI_REGDEF_H
#define HW_ARM_STM32G070_EXTI_REGDEF_H

enum reg_index {
	RI_RTSR,
	RI_FTSR,
	RI_SWIER,
	RI_RPR,
	RI_FPR,
	RI_EXTICR_BEGIN = 0x60U/4U,
	RI_EXTICR1 = RI_EXTICR_BEGIN,
	RI_EXTICR2,
	RI_EXTICR3,
	RI_EXTICR4,
	RI_EXTICR_END,
	RI_IMR = 0x80U/4U,
	RI_EMR,
	RI_END
};

#endif
