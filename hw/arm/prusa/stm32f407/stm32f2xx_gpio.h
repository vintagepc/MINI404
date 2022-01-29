/*-
 * QEMU model of the stm32f2xx GPIO module
 * Portions Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
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
#ifndef STM32F2XX_GPIO_H
#define STM32F2XX_GPIO_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"
#include "../stm32_common/stm32_common.h"

#define STM32_GPIO_MODER   (0x00 / 4)
#define STM32_GPIO_OTYPER  (0x04 / 4)
#define STM32_GPIO_OSPEEDR (0x08 / 4)
#define STM32_GPIO_PUPDR   (0x0c / 4)
#define STM32_GPIO_IDR     (0x10 / 4)
#define STM32_GPIO_ODR     (0x14 / 4)
#define STM32_GPIO_BSRR    (0x18 / 4)
#define STM32_GPIO_LCKR    (0x1c / 4)
#define STM32_GPIO_AFRL    (0x20 / 4)
#define STM32_GPIO_AFRH    (0x24 / 4)
#define STM32_GPIO_MAX     (0x28 / 4)

#define STM32_GPIO_PIN_COUNT 16

#define TYPE_STM32F2XX_GPIO "stm32f2xx-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f2xx_gpio, STM32F2XX_GPIO)


/* GPIO - f2xx */
void f2xx_gpio_exti_set(stm32f2xx_gpio *, unsigned, qemu_irq);
void f2xx_gpio_wake_set(stm32f2xx_gpio *, unsigned, qemu_irq);


struct stm32f2xx_gpio {
    STM32Peripheral parent;
    MemoryRegion iomem;

    uint32_t idr_mask;

    qemu_irq pin[STM32_GPIO_PIN_COUNT];
    qemu_irq exti[STM32_GPIO_PIN_COUNT];
    qemu_irq alternate_function[STM32_GPIO_PIN_COUNT];
    qemu_irq cpu_wake[STM32_GPIO_PIN_COUNT];

    uint32_t regs[STM32_GPIO_MAX];
    uint32_t ccr;
};

#endif //#ifndef STM32F2XX_GPIO_H
