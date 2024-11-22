/*
    stm32h503xx.h - Chip variant definition for the STM32H503xx series.

	Copyright 2024 VintagePC <https://github.com/vintagepc/>

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

#ifndef HW_ARM_PRUSA_STM32H503XX_H
#define HW_ARM_PRUSA_STM32H503XX_H

#include "../stm32_common/stm32_chip_macros.h"
#include "../stm32_common/stm32_types.h"
#include "../utility/macros.h"
#include "qemu/units.h"
#include "../stm32f407/stm32f2xx_tim.h"

#include "../stm32_registers/generated/stm32h503/Addresses.h"
#include "../stm32_registers/generated/stm32h503/IRQs.h"

static const stm32_soc_cfg_t stm32h503xx_cfg =
{
	.name = "STM32H503",
	.nvic_irqs = H503_COUNT_IRQ,
	.rcc_hse_freq = 24*MHz,
	.rcc_hsi_freq = 64*MHz,
	.rcc_lse_freq = 32768,
	.rcc_lsi_freq = 32*KHz,
	.flash_base = H503_FLASH_ADDR,
	.flash_variants = {
		{TYPE_STM32H503xx_SOC, 128U*KiB},
		{NULL}
	},
	.sram_base = H503_SRAM1_ADDR,
	.sram_variants = {
		{TYPE_STM32H503xx_SOC, 32U*KiB},
		{NULL}
	},
	.ccmsram_base = 0x10000000,
	.ccmsram_variants = {
		{TYPE_STM32H503xx_SOC, 0*KiB},
		{NULL}
	},
	.perhipherals = {
		PER_LNI(P_TIM1, TYPE_STM32F4XX_TIMER, H503_TIM1_ADDR, IRQ_SKIP_CONNECT, H503_TIM1_UP_IRQ, H503_TIM1_TRG_COM_IRQ, H503_TIM1_BRK_IRQ, H503_TIM1_CC_IRQ),
		PER_LNI(P_TIM2, TYPE_STM32F4XX_TIMER, H503_TIM2_ADDR, H503_TIM2_IRQ),
		PER_LNI(P_TIM3, TYPE_STM32F4XX_TIMER, H503_TIM3_ADDR, H503_TIM3_IRQ),
		PER_LNI(P_TIM6, TYPE_STM32F4XX_TIMER, H503_TIM6_ADDR, H503_TIM6_IRQ),
		PER_LNI(P_TIM7, TYPE_STM32F4XX_TIMER, H503_TIM7_ADDR, H503_TIM7_IRQ),

		PER_LNIA(USART, 1, H503),
		PER_LNIA(USART, 2, H503),
		PER_LNIA(USART, 3, H503),

		PER_LNA(GPIO, A, H503),
		PER_LNA(GPIO, B, H503),
		PER_LNA(GPIO, C, H503),
		PER_LNA(GPIO, D, H503),
		PER_LNA(GPIO, H, H503),

		PER_LNI(P_DMA1, TYPE_STM32H503_DMA, H503_GPDMA1_ADDR, H503_GPDMA1_Channel0_IRQ, H503_GPDMA1_Channel1_IRQ, H503_GPDMA1_Channel2_IRQ, H503_GPDMA1_Channel3_IRQ, H503_GPDMA1_Channel4_IRQ, H503_GPDMA1_Channel5_IRQ, H503_GPDMA1_Channel6_IRQ, H503_GPDMA1_Channel7_IRQ),
		PER_LNI(P_DMA2, TYPE_STM32H503_DMA, H503_GPDMA2_ADDR, H503_GPDMA2_Channel0_IRQ, H503_GPDMA2_Channel1_IRQ, H503_GPDMA2_Channel2_IRQ, H503_GPDMA2_Channel3_IRQ, H503_GPDMA2_Channel4_IRQ, H503_GPDMA2_Channel5_IRQ, H503_GPDMA2_Channel6_IRQ, H503_GPDMA2_Channel7_IRQ),

		// PER_LNI(P_I2C1, NULL, 0x40005400, G070_I2C1_IRQ),
		// PER_LNI(P_I2C2, NULL, 0x40005800, G070_I2C2_3_IRQ),
		// PER_LNI(P_I2C3, NULL, 0x40008800, G070_I2C2_3_IRQ),

		// PER_LNI(P_SPI1, TYPE_STM32G070_SPI, 0x40013000, G070_SPI1_IRQ),
		// PER_LNI(P_SPI2, TYPE_STM32G070_SPI, 0x40003800, G070_SPI2_3_IRQ),
		// PER_LNI(P_SPI3, TYPE_STM32G070_SPI, 0x40003C00, G070_SPI2_3_IRQ),

		// PER_LNI(P_ADC1, TYPE_STM32G070_ADC, 0x40012400, G070_ADC_IRQ),

		PER_LNIA(ADC,1,H503),
		PER_LN(P_ADCC,TYPE_STM32H503_ADCC, H503_ADC12_COMMON_ADDR + 0x08),
		PER_LNA(ICACHE, ,H503),

		PER_LNIF(P_RCC, TYPE_STM32H503_RCC, H503_RCC_ADDR, PERIPH_CFG_FLAG_NON_STM32P, H503_RCC_IRQ),
		PER_LNI(P_FINT, TYPE_STM32G070_FINT, H503_FLASH_R_ADDR, H503_FLASH_IRQ),
		PER_LNA(IWDG, , H503),
		PER_LNA(CRC, ,H503),
		PER_LNA(RNG, ,H503),
		// PER_LN(P_SYSCFG, TYPE_STM32G070_SYSCFG, 0x40010000),

		// PER_LN(P_OTP, TYPE_STM32G070_OTP, 0x1FFF7000),
		PER_LNA(PWR, ,H503)

		// PER_LNI(P_DMAMUX, TYPE_STM32G070_DMAMUX, 0x40020800, G070_DMA1_CH4_7_DMAMUX_DMA2_CH1_5_IRQ),

		// PER_LNI(P_EXTI, TYPE_STM32G070_EXTI, 0x40021800, [0 ... 1] = G070_EXTI0_1_IRQ, [2 ... 3] = G070_EXTI2_3_IRQ, [4 ... 15] = G070_EXTI4_15_IRQ),
		// PER_LN(P_DBG, TYPE_STM32G070_DBG, 0x40015800),
	}
} ;

#endif // HW_ARM_PRUSA_STM32G070XX_H
