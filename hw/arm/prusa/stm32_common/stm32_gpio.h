/*-
 * GPIO module for STM32 SOCs in QEMU.
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
#ifndef STM32_COMMON_GPIO_H
#define STM32_COMMON_GPIO_H

#include "stm32_types.h"

typedef struct COM_STRUCT_NAME(Gpio) COM_STRUCT_NAME(Gpio);

void stm32_common_gpio_wake_set(COM_STRUCT_NAME(Gpio) *, unsigned, qemu_irq);

#endif //#ifndef STM32_COMMON_GPIO_H
