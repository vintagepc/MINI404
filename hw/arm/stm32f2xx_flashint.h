

#ifndef STM32F2XX_FINT_H
#define STM32F2XX_FINT_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu-common.h"

#define STM32_FINT_ACR     (0x00 / 4)
#define STM32_FINT_KEYR    (0x04 / 4)
#define STM32_FINT_OPTKEYR (0x08 / 4)
#define STM32_FINT_SR      (0x0c / 4)
#define STM32_FINT_CR     (0x10 / 4)
#define STM32_FINT_OPTCR     (0x14 / 4)
#define STM32_FINT_MAX     (0x18 / 4)

#define TYPE_STM32F2XX_FINT "stm32f2xx-fint"
OBJECT_DECLARE_SIMPLE_TYPE(stm32f2xx_fint, STM32F2XX_FINT)

struct stm32f2xx_fint {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[STM32_FINT_MAX];
};

#endif //#ifndef STM32F2XX_FINT_H
