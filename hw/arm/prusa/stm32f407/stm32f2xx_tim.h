#ifndef STM32F2XX_TIM_H
#define STM32F2XX_TIM_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "stm32_util.h"


#define R_TIM_MAX    (0x54 / 4)

#define TYPE_STM32F4XX_TIMER "stm32f4xx-timer"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_tim, STM32F4XX_TIMER)

struct f2xx_tim {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t regs[R_TIM_MAX];
    uint8_t id;
    qemu_irq pwm_ratio_changed;
    qemu_irq pwm_enable;
};

#endif // STM32F2XX_TIM_H
