/*
    stm32f030xx.h - Chip variant definition for the STM32F030xx

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

#ifndef HW_ARM_PRUSA_STM32F030XX_H
#define HW_ARM_PRUSA_STM32F030XX_H

#include "../stm32_common/stm32_chip_macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_registers/generated/stm32f030/Addresses.h"
#include "../stm32_registers/generated/stm32f030/IRQs.h"
#include "hw/misc/stm32f4xx_exti.h"
#include "../stm32f407/stm32f2xx_rtc.h"
#include "../stm32f407/stm32f2xx_tim.h"
#include "../stm32f407/stm32f2xx_pwr.h"
#include "../utility/macros.h"
#include "qemu/units.h"

static const stm32_soc_cfg_t stm32f030xx_cfg =
{
	.nvic_irqs = F030_COUNT_IRQ,
	.rcc_hse_freq = 8*MHz,
	.rcc_hsi_freq = 8*MHz,
	.rcc_lse_freq = 32768,
	.rcc_lsi_freq = 40*KHz,
	.flash_base = F030_FLASH_ADDR,
	.flash_variants = {
		{TYPE_STM32F030x4_SOC, 16U*KiB},
		{TYPE_STM32F030x6_SOC, 32U*KiB},
		{TYPE_STM32F030x8_SOC, 64U*KiB},
		{TYPE_STM32F030xC_SOC, 256U*KiB},
		{NULL}
	},
	.sram_base = F030_SRAM_ADDR,
	.sram_variants = {
		{TYPE_STM32F030x4_SOC, 4U*KiB},
		{TYPE_STM32F030x6_SOC, 4U*KiB},
		{TYPE_STM32F030x8_SOC, 8U*KiB},
		{TYPE_STM32F030xC_SOC, 32U*KiB},
		{NULL}
	},
	.ccmsram_base = 0x10000000,
	.ccmsram_variants = {
		{TYPE_STM32F030x4_SOC, 64U*KiB},
		{TYPE_STM32F030x6_SOC, 64U*KiB},
		{TYPE_STM32F030x8_SOC, 64U*KiB},
		{TYPE_STM32F030xC_SOC, 64U*KiB},
		{NULL}
	},
	.perhipherals = {
		PER_LNIA(USART, 1, F030),
		PER_LNIA(USART, 2, F030),

		PER_LNIA(SPI, 1, F030),
		PER_LNIA(SPI, 2, F030),

		PER_LNA(GPIO, A, F030),
		PER_LNA(GPIO, B, F030),
		PER_LNA(GPIO, C, F030),
		PER_LNA(GPIO, D, F030),
		PER_LNA(GPIO, F, F030),

		PER_LNA(DMA,1,F030),

		PER_LNA(SYSCFG,,F030),

		PER_LNI(P_TIM1,  TYPE_STM32F4XX_TIMER, F030_TIM1_ADDR , F030_TIM1_BRK_UP_TRG_COM_IRQ),
		PER_LNI(P_TIM3,  TYPE_STM32F4XX_TIMER, F030_TIM3_ADDR , F030_TIM3_IRQ),
		PER_LNI(P_TIM6,  TYPE_STM32F4XX_TIMER, F030_TIM6_ADDR , F030_TIM6_IRQ),
		PER_LNI(P_TIM14, TYPE_STM32F4XX_TIMER, F030_TIM14_ADDR, F030_TIM14_IRQ),
		PER_LNI(P_TIM15, TYPE_STM32F4XX_TIMER, F030_TIM15_ADDR, F030_TIM15_IRQ),
		PER_LNI(P_TIM16, TYPE_STM32F4XX_TIMER, F030_TIM16_ADDR, F030_TIM16_IRQ),
		PER_LNI(P_TIM17, TYPE_STM32F4XX_TIMER, F030_TIM17_ADDR, F030_TIM17_IRQ),

	 	PER_LNI(P_I2C1, NULL, F030_I2C1_ADDR, F030_I2C1_IRQ),
	 	PER_LNI(P_I2C2, NULL, F030_I2C2_ADDR, F030_I2C2_IRQ),

		PER_LNIA(ADC, 1, F030),
		PER_LN(P_ADCC, TYPE_STM32F030_ADCC, F030_ADC_ADDR),

		PER_LNIF(P_RCC, TYPE_STM32F030_RCC, F030_RCC_ADDR, PERIPH_CFG_FLAG_NON_STM32P, F030_RCC_IRQ),
		PER_LNI(P_FINT, TYPE_STM32F40x_F41x_FINT, F030_FLASH_R_ADDR, F030_FLASH_IRQ),
		PER_LNA(IWDG, , F030),
		PER_LNA(CRC,, F030),

		PER_LNI(P_EXTI, TYPE_STM32F4XX_EXTI, F030_EXTI_ADDR, PERIPH_CFG_FLAG_NON_STM32P, [0 ... 1] = F030_EXTI0_1_IRQ, [2 ... 3] = F030_EXTI2_3_IRQ, [4 ... 15] = F030_EXTI4_15_IRQ),
	}
} ;

#endif // HW_ARM_PRUSA_STM32F030XX_H
