/*
    stm32c092xx.h - Chip variant definition for the STM32C092xx

	Copyright 2026 VintagePC <https://github.com/vintagepc/>

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

#ifndef HW_ARM_PRUSA_STM32C092XX_H
#define HW_ARM_PRUSA_STM32C092XX_H

#include "../stm32_common/stm32_chip_macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_registers/generated/stm32c092/Addresses.h"
#include "../stm32_registers/generated/stm32c092/IRQs.h"
#include "../utility/macros.h"
#include "qemu/units.h"

static const stm32_soc_cfg_t stm32c092xx_cfg =
{
	.nvic_irqs = C092_COUNT_IRQ,
	.rcc_hse_freq = 48*MHz,
	.rcc_hsi_freq = 48*MHz,
	.rcc_lse_freq = 32768,
	.rcc_lsi_freq = 32*KHz,
	.flash_base = C092_FLASH_ADDR,
	.flash_variants = {
		{TYPE_STM32C092xB_SOC, 128U*KiB},
		{TYPE_STM32C092xC_SOC, 256U*KiB},
		{NULL}
	},
	.sram_base = C092_SRAM_ADDR,
	.sram_variants = {
		{TYPE_STM32C092xB_SOC, 30U*KiB},
		{TYPE_STM32C092xC_SOC, 30U*KiB},
		{NULL}
	},
	.ccmsram_base = 0,
	.ccmsram_variants = {
		{NULL}
	},
	.perhipherals = {

	}
} ;

#endif // HW_ARM_PRUSA_STM32C092XX_H
