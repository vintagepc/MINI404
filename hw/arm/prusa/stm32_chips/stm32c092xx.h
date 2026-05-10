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
#include "../stm32f407/stm32f2xx_tim.h"
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
		PER_LNI(P_CAN1, NULL, C092_FDCAN1_ADDR, IRQ_SKIP_CONNECT, C092_FDCAN1_IT0_IRQ, C092_FDCAN1_IT1_IRQ),
		PER_LNA(CRC,, C092),
		PER_LNI(P_I2C1, TYPE_STM32C092_I2C, C092_I2C1_ADDR, C092_I2C1_IRQ),
		PER_LNI(P_I2C2, TYPE_STM32C092_I2C, C092_I2C2_ADDR, C092_I2C2_IRQ),
		PER_LN(P_GPIOA, TYPE_STM32C092_GPIO, C092_GPIOA_ADDR),
		PER_LN(P_GPIOB, TYPE_STM32C092_GPIO, C092_GPIOB_ADDR),
		PER_LN(P_GPIOC, TYPE_STM32C092_GPIO, C092_GPIOC_ADDR),
		PER_LN(P_GPIOD, TYPE_STM32C092_GPIO, C092_GPIOD_ADDR),
		PER_LN(P_GPIOF, TYPE_STM32C092_GPIO, C092_GPIOF_ADDR),
		PER_LNA(IWDG, , C092),
		PER_LN(P_OTP, TYPE_STM32C092_OTP, 0x1FFF7000),
		PER_LNIF(P_RCC, TYPE_STM32C092_RCC, C092_RCC_ADDR, PERIPH_CFG_FLAG_NON_STM32P, C092_RCC_IRQ),
		PER_LNI(P_TIM1, TYPE_STM32F4XX_TIMER, C092_TIM1_ADDR, IRQ_SKIP_CONNECT, C092_TIM1_BRK_UP_TRG_COM_IRQ, C092_TIM1_BRK_UP_TRG_COM_IRQ, C092_TIM1_BRK_UP_TRG_COM_IRQ, C092_TIM1_CC_IRQ),
		PER_LNI(P_TIM2, TYPE_STM32F4XX_TIMER, C092_TIM2_ADDR, C092_TIM2_IRQ),
		PER_LNI(P_TIM3, TYPE_STM32F4XX_TIMER, C092_TIM3_ADDR, C092_TIM3_IRQ),
		PER_LNI(P_TIM14, TYPE_STM32F4XX_TIMER, C092_TIM14_ADDR, C092_TIM14_IRQ),
		PER_LNI(P_TIM15, TYPE_STM32F4XX_TIMER, C092_TIM15_ADDR, C092_TIM15_IRQ),
		PER_LNI(P_TIM16, TYPE_STM32F4XX_TIMER, C092_TIM16_ADDR, C092_TIM16_IRQ),
		PER_LNI(P_TIM17, TYPE_STM32F4XX_TIMER, C092_TIM17_ADDR, C092_TIM17_IRQ),
		PER_LNA(SYSCFG,,C092),
	}
} ;

#endif // HW_ARM_PRUSA_STM32C092XX_H
