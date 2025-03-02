/*-
 * GPIO module register data for STM32 SOCs in QEMU.
 * Note this data is also shared with the unit test for the GPIO peripheral.
 * Portions Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
 * Reworked for more chips in 2022 by VintagePC. Currently the layout is known
 * to be similar for:
 * 	- F030 series,
 *	- G070 series,
 *  - F2xx series (given it originated from the Pebble QEMU project)
 *	- F4xx series (specifically, 407, 427 and 429 are known)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef STM32_COMMON_GPIO_DATA_H
#define STM32_COMMON_GPIO_DATA_H

#include "stm32_shared.h"

enum {
    R_MODE_INPUT = 0,
    R_MODE_OUTPUT,
    R_MODE_ALT,
    R_MODE_ANALOG
};

enum {
    R_PUPD_NONE = 0,
    R_PUPD_PU,
    R_PUPD_PD,
};

enum {
    R_OTYPE_PP = 0,
    R_OTYPE_OD,
};

enum reg_index {
	RI_MODER,
	RI_OTYPER,
	RI_OSPEEDR,
	RI_PUPDR,
	RI_IDR,
	RI_ODR,
	RI_BSRR,
	RI_LCKR,
	RI_AFRL,
	RI_AFRH,
	RI_BRR,
	RI_END,
	RI_BANK_SIZE = RI_END,
};

#define MAX_GPIO_BANKS (STM32_P_GPIO_END - STM32_P_GPIO_BEGIN)

enum {
	BANK_A,
	BANK_B,
	BANK_C,
	BANK_D,
	BANK_E,
	BANK_F,
	BANK_G,
	BANK_H,
	BANK_I,
	BANK_J,
	BANK_K,
	BANK_MAX,
};

QEMU_BUILD_BUG_MSG(MAX_GPIO_BANKS != BANK_MAX, "GPIO maximum bank index has changed and is not consistent!");

static const stm32_reginfo_t stm32f030_gpio_bank_a[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX, .reset_val = 0x28000000 },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX, .reset_val = 0x0C000000 },
	[RI_PUPDR] = {.mask = UINT32_MAX, .reset_val = 0x24000000 },
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t stm32f030_gpio_bank_b[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX},
	[RI_PUPDR] = {.mask = UINT32_MAX},
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t stm32_gpio_bank_all_zero[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX },
	[RI_PUPDR] = {.mask = UINT32_MAX },
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.is_reserved = true},
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t stm32_gpio_bank_not_present[RI_END] =
{
	[RI_MODER ... RI_BRR] = {.is_reserved = true},
};

static const stm32_reginfo_t* stm32f030_gpio_reginfo[MAX_GPIO_BANKS] =
{
	[BANK_A] = stm32f030_gpio_bank_a,
	[BANK_B] = stm32f030_gpio_bank_b,
	[BANK_C ... BANK_D] = stm32_gpio_bank_all_zero,
	[BANK_E] = stm32_gpio_bank_not_present,
	[BANK_F] = stm32_gpio_bank_all_zero,
	[BANK_G ... BANK_K] = stm32_gpio_bank_not_present,

};

static const stm32_reginfo_t stm32g070_gpio_bank_a[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX, .reset_val = 0xEBFFFFFF },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX, .reset_val = 0x0C000000 },
	[RI_PUPDR] = {.mask = UINT32_MAX, .reset_val = 0x24000000 },
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};
static const stm32_reginfo_t stm32g070_gpio_bank_b_f[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX, .reset_val = 0xFFFFFFFF },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX},
	[RI_PUPDR] = {.mask = UINT32_MAX},
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t* stm32g070_gpio_reginfo[RI_END] =
{
	[BANK_A] = stm32g070_gpio_bank_a,
	[BANK_B ... BANK_F] = stm32g070_gpio_bank_b_f,
	[BANK_G ... BANK_K] = stm32_gpio_bank_not_present,
};

static const stm32_reginfo_t stm32f2xx_f4xx_gpio_bank_a[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX, .reset_val = 0xA8000000 },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX, .reset_val = 0x0C000000 },
	[RI_PUPDR] = {.mask = UINT32_MAX, .reset_val = 0x64000000 },
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t stm32f2xx_f4xx_gpio_bank_b[RI_END] =
{
	[RI_MODER] = {.mask = UINT32_MAX, .reset_val = 0x00000280 },
	[RI_OTYPER] = {.mask = UINT16_MAX},
	[RI_OSPEEDR] = {.mask = UINT32_MAX, .reset_val = 0x000000C0 },
	[RI_PUPDR] = {.mask = UINT32_MAX, .reset_val = 0x00000100 },
	[RI_IDR] = {.mask = UINT16_MAX},
	[RI_ODR] = {.mask = UINT16_MAX},
	[RI_BSRR] = {.mask = UINT32_MAX},
	[RI_LCKR] = {.mask = (1U << 17U) - 1U },
	[RI_AFRL] = {.mask = UINT32_MAX},
	[RI_AFRH] = {.mask = UINT32_MAX},
	[RI_BRR] = {.mask = UINT16_MAX},
};

static const stm32_reginfo_t* stm32f2xx_gpio_reginfo[RI_END] =
{
	[BANK_A] = stm32f2xx_f4xx_gpio_bank_a,
	[BANK_B] = stm32f2xx_f4xx_gpio_bank_b,
	[BANK_C ... BANK_K] = stm32_gpio_bank_all_zero,
};

static const stm32_reginfo_t* stm32f4xx_gpio_reginfo[RI_END] =
{
	[BANK_A] = stm32f2xx_f4xx_gpio_bank_a,
	[BANK_B] = stm32f2xx_f4xx_gpio_bank_b,
	[BANK_C ... BANK_K] = stm32_gpio_bank_all_zero,
};

#endif
