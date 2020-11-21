#ifndef STM32F2XX_CRC_H
#define STM32F2XX_CRC_H

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"

#define TYPE_STM32F2XX_CRC "stm32f2xx-crc"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_crc,STM32F2XX_CRC);

typedef struct f2xx_crc {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t crc;
    uint8_t idr;
} f2xx_crc;

#endif // STM32F2XX_CRC_H
