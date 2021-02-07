#ifndef STM32F4XX_IWDG_H
#define STM32F4XX_IWDG_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "exec/memory.h"

#define R_IWDG_MAX 4

typedef struct Stm32Rcc Stm32Rcc;

#define TYPE_STM32F4XX_IWDG "stm32f4xx-iwdg"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f4xx_iwdg, STM32F4XX_IWDG);

typedef struct stm32f4xx_iwdg {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    union {
        uint32_t all[R_IWDG_MAX];
        struct {
            union {
                uint32_t KEY :16;
                uint32_t :16;
            }KR QEMU_PACKED;
            union {
                uint32_t PR :3;
                uint32_t :29;
            }PR QEMU_PACKED;
            union {
                uint32_t RL :12;
                uint32_t :20;
            }RLR QEMU_PACKED;
            union {
                uint32_t PVU :1;
                uint32_t RVU :1;
                uint32_t :30;
            }SR QEMU_PACKED;
        } defs;
    } regs QEMU_PACKED;

    QEMUTimer *timer;

    bool time_changed, started;

    Stm32Rcc *rcc; // RCC for clock speed. 

} stm32f4xx_iwdg;

#endif //STM32F4XX_IWDG_H
