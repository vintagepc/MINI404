/*
    stm32_rng.c - RNG block for STM32F4xx

	Copyright 2021-4 VintagePC <https://github.com/vintagepc/>

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

#ifndef STM32_RNG_H
#define STM32_RNG_H

enum stm32_rng_ri
{
	RI_CR                   = (0x00 /4U), /* RNG control register, */
	RI_SR                   = (0x04 /4U), /* RNG status register, */
	RI_DR                   = (0x08 /4U), /* RNG data register, */
	RI_HTCR                 = (0x10 /4U), /* RNG health test configuration register, */
	RI_END
};


#endif //STM32_RNG_H
