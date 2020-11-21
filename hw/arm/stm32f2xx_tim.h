#ifndef STM32F2XX_TIM_H
#define STM32F2XX_TIM_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "stm32_util.h"

#define R_TIM_CR1    (0x00 / 4) //p
#define R_TIM_CR2    (0x04 / 4)
#define R_TIM_SMCR   (0x08 / 4)
#define R_TIM_DIER   (0x0c / 4) //p
#define R_TIM_SR     (0x10 / 4) //p
#define R_TIM_EGR    (0x14 / 4) //p
#define R_TIM_CCMR1  (0x18 / 4)
#define R_TIM_CCMR2  (0x1c / 4)
#define R_TIM_CCER   (0x20 / 4)
#define R_TIM_CNT    (0x24 / 4)
#define R_TIM_PSC    (0x28 / 4) //p
#define R_TIM_ARR    (0x2c / 4) //p
#define R_TIM_CCR1   (0x34 / 4)
#define R_TIM_CCR2   (0x38 / 4)
#define R_TIM_CCR3   (0x3c / 4)
#define R_TIM_CCR4   (0x40 / 4)
#define R_TIM_DCR    (0x48 / 4)
#define R_TIM_DMAR   (0x4c / 4)
#define R_TIM_OR     (0x50 / 4)
#define R_TIM_MAX    (0x54 / 4)

static const char *f2xx_tim_reg_names[] = {
    ENUM_STRING(R_TIM_CR1),
    ENUM_STRING(R_TIM_CR2),
    ENUM_STRING(R_TIM_SMCR),
    ENUM_STRING(R_TIM_DIER),
    ENUM_STRING(R_TIM_SR),
    ENUM_STRING(R_TIM_EGR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_CCMR2),
    ENUM_STRING(R_TIM_CCER),
    ENUM_STRING(R_TIM_CNT),
    ENUM_STRING(R_TIM_PSC),
    ENUM_STRING(R_TIM_ARR),
    ENUM_STRING(R_TIM_CCR1),
    ENUM_STRING(R_TIM_CCR2),
    ENUM_STRING(R_TIM_CCR3),
    ENUM_STRING(R_TIM_CCR4),
    ENUM_STRING(R_TIM_DCR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_DMAR),
    ENUM_STRING(R_TIM_OR)
};

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
