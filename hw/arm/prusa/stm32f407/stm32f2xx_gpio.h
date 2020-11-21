

#ifndef STM32F2XX_GPIO_H
#define STM32F2XX_GPIO_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"

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

struct stm32f2xx_gpio {
    SysBusDevice busdev;
    MemoryRegion iomem;

   // stm32_periph_t periph;
    uint32_t periph;
    uint32_t idr_mask;

    qemu_irq pin[STM32_GPIO_PIN_COUNT];
    qemu_irq exti[STM32_GPIO_PIN_COUNT];
    qemu_irq alternate_function[STM32_GPIO_PIN_COUNT];
    qemu_irq cpu_wake[STM32_GPIO_PIN_COUNT];

    uint32_t regs[STM32_GPIO_MAX];
    uint32_t ccr;
};

#endif //#ifndef STM32F2XX_GPIO_H
