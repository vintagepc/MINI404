/*
    stm32_iwdg_regdata.h - IWDG register data for STM32.
	Currently supports the F4xx, F030, and G070 layouts.
	Also used by the unit test cases.

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

#ifndef HW_ARM_STM32_IWDG_REGDEF_H
#define HW_ARM_STM32_IWDG_REGDEF_H

enum reg_index {
	RI_KR,
	RI_PR,
	RI_RLR,
	RI_SR,
	RI_WINR,
	RI_END
};

static const uint16_t IWDG_PRESCALES[8] = { 4, 8, 16, 32, 64, 128, 256, 256 };

#endif
