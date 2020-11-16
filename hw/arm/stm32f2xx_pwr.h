#ifndef STM32F2XX_PWR_H
#define STM32F2XX_PWR_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"

#define R_PWR_CR      (0x00/4)
#define R_PWR_CR_LPDS   0x00000001
#define R_PWR_CR_PDDS   0x00000002

#define R_PWR_CSR     (0x04/4)

#define R_PWR_MAX     (0x08/4)

#define TYPE_STM32F2XX_PWR "stm32f2xx-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_pwr, STM32F2XX_PWR);

typedef struct f2xx_pwr {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    uint32_t      regs[R_PWR_MAX];
} f2xx_pwr;

#endif //STM32F2XX_PWR_H
