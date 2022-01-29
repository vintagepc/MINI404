/*
    stm32_rcc_if.h  - Common STM32 RCC parent interface functions
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

#ifndef STM32_RCC_IF_H
#define STM32_RCC_IF_H

typedef struct STM32Peripheral STM32Peripheral;
typedef struct COM_STRUCT_NAME(Rcc) COM_STRUCT_NAME(Rcc);

// Returns the clock frequency for the peripheral.
extern uint32_t stm32_rcc_if_get_periph_freq(STM32Peripheral *p);

// Checks if a clock is enabled and prints a guest error if not
extern bool stm32_rcc_if_check_periph_clk(STM32Peripheral *p);

/* Sets the IRQ to be called when the specified peripheral clock changes
 * frequency. */
extern void stm32_rcc_if_set_periph_clk_irq(
        STM32Peripheral *p,
        qemu_irq periph_irq);

// Helpers for dissecting a register of enable and reset bits:

// Dissects the mask and sets the clock enable for the given peripheral bits.
extern void stm32_common_rcc_enable_write(COM_STRUCT_NAME(Rcc) *s, uint32_t mask, const uint8_t (*periphs)[32]);

// Dissects the mask and triggers the reset IRQ for the given peripheral bits.
extern void stm32_common_rcc_reset_write(COM_STRUCT_NAME(Rcc) *s, uint32_t mask, const uint8_t (*periphs)[32]);

#endif // STM32_RCC_IF
