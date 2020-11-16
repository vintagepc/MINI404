
// (C) VintagePC 2020

#ifndef STM32F2XX_ITM_H
#define STM32F2XX_ITM_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"

#define R_ITM_PORT_BASE     (0x00 / 4)
#define R_ITM_TER   (0xE00 / 4)
#define R_ITM_TPR   (0xE40 / 4)
#define R_ITM_TCR   (0xE80 / 4)
#define R_ITM_LAR     (0xFB0 / 4)
#define R_ITM_MAX     (0xFFC / 4)

#define TYPE_STM32F4XX_ITM "stm32f4xx-itm"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f4xx_itm, STM32F4XX_ITM)

struct stm32f4xx_itm {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[R_ITM_MAX];
    uint32_t port_buffer[101];
    uint8_t buffer_pos;
    bool unlocked; // Set when LAR is written properly
};

#endif //#ifndef STM32F2XX_ITM_H
