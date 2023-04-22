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
#include "hw/misc/stm32f4xx_exti.h"
#include "../stm32f407/stm32f2xx_rtc.h"
#include "../stm32f407/stm32f2xx_tim.h"
#include "../stm32f407/stm32f4xx_rng.h"
#include "../stm32f407/stm32f2xx_pwr.h"
#include "../utility/macros.h"
#include "qemu/units.h"
// IRQs:
#define _I(x) _JOIN3R(F030_, x, _IRQ)

enum F030_IRQ{
	_I(WWDG) = 0,
	_I(RTC) = 2,
	_I(FLASH),
	_I(RCC),
	_I(EXTI0_1),
	_I(EXTI2_3),
	_I(EXTI4_15) = 9,
	_I(DMA_CH1),
	_I(DMA_CH2_3),
	_I(DMA_CH4_5),
	_I(ADC),
	_I(TIM1_BRK_UP_TRG_COM),
	_I(TIM1_CC),
	_I(TIM3) = 16,
	_I(TIM6),
	_I(TIM14) = 19,
	_I(TIM15),
	_I(TIM16),
	_I(TIM17),
	_I(I2C1),
	_I(I2C2),
	_I(SPI1),
	_I(SPI2),
	_I(USART1),
	_I(USART2),
	_I(USART_3_4_5_6),
	_I(USB) = 31,
	_I(COUNT), // IRQ size tracking.
};
#undef _I

static const stm32_soc_cfg_t stm32f030xx_cfg =
{
	.nvic_irqs = F030_COUNT_IRQ,
	.rcc_hse_freq = 8000000,
	.rcc_hsi_freq = 8000000,
	.rcc_lse_freq = 32768,
	.rcc_lsi_freq = 40000,
	.flash_base = 0x08000000,
	.flash_variants = {
		{TYPE_STM32F030x4_SOC, 16U*KiB},
		{TYPE_STM32F030x6_SOC, 32U*KiB},
		{TYPE_STM32F030x8_SOC, 64U*KiB},
		{TYPE_STM32F030xC_SOC, 256U*KiB},
		{NULL}
	},
	.sram_base = 0x20000000,
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
		PER_LNI(P_UART1, TYPE_STM32F030_USART, 0x40013800, F030_USART1_IRQ),
		PER_LNI(P_UART2, TYPE_STM32F030_USART, 0x40004400, F030_USART2_IRQ),

		PER_LNI(P_SPI1, TYPE_STM32F030_SPI, 0x40013000, F030_SPI1_IRQ),
		PER_LNI(P_SPI2, TYPE_STM32F030_SPI, 0x40003800, F030_SPI2_IRQ),

		PER_LN(P_GPIOA, TYPE_STM32F030_GPIO, 0x48000000),
		PER_LN(P_GPIOB, TYPE_STM32F030_GPIO, 0x48000400),
		PER_LN(P_GPIOC, TYPE_STM32F030_GPIO, 0x48000800),
		PER_LN(P_GPIOD, TYPE_STM32F030_GPIO, 0x48000C00),
		PER_LN(P_GPIOF, TYPE_STM32F030_GPIO, 0x48001400),

		PER_LN(P_DMA1, TYPE_STM32F030_DMA, 0x40020000),

		PER_LN(P_SYSCFG, TYPE_STM32F030_SYSCFG, 0x40010000),

		PER_LNI(P_TIM1, TYPE_STM32F4XX_TIMER, 0x40012C00, F030_TIM1_BRK_UP_TRG_COM_IRQ),
		PER_LNI(P_TIM3, TYPE_STM32F4XX_TIMER, 0x40000400, F030_TIM3_IRQ),
		PER_LNI(P_TIM6, TYPE_STM32F4XX_TIMER, 0x40001000, F030_TIM6_IRQ),
		PER_LNI(P_TIM14, TYPE_STM32F4XX_TIMER, 0x40002000, F030_TIM14_IRQ),
		PER_LNI(P_TIM15, TYPE_STM32F4XX_TIMER, 0x40014000, F030_TIM15_IRQ),
		PER_LNI(P_TIM16, TYPE_STM32F4XX_TIMER, 0x40014400, F030_TIM16_IRQ),
		PER_LNI(P_TIM17, TYPE_STM32F4XX_TIMER, 0x40014800, F030_TIM17_IRQ),

	 	PER_LNI(P_I2C1, NULL, 0x40005400, F030_I2C1_IRQ),
	 	PER_LNI(P_I2C2, NULL, 0x40005800, F030_I2C2_IRQ),

		PER_LNI(P_ADC1, TYPE_STM32F030_ADC, 0x40012400, F030_ADC_IRQ),
		PER_LN(P_ADCC, TYPE_STM32F030_ADCC, 0x40012400 + 0x308),

		PER_LNI(P_RCC, TYPE_STM32F030_RCC, 0x40021000, F030_RCC_IRQ),
		PER_LNI(P_FINT, TYPE_STM32F40x_F41x_FINT, 0x40022000, F030_FLASH_IRQ),
		PER_LN(P_IWDG, TYPE_STM32F030_IWDG, 0x40003000),
		PER_LN(P_CRC, TYPE_STM32F030_CRC, 0x40023000),

		PER_LNI(P_EXTI, TYPE_STM32F4XX_EXTI, 0x40010400, [0 ... 1] = F030_EXTI0_1_IRQ, [2 ... 3] = F030_EXTI2_3_IRQ, [4 ... 15] = F030_EXTI4_15_IRQ),
	}
} ;

#endif // HW_ARM_PRUSA_STM32F030XX_H
