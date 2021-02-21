#ifndef STM32F4XX_OTP_H
#define STM32F4XX_OTP_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "sysemu/block-backend.h"


#define OTP_SIZE 0x220/4

#define TYPE_STM32F4XX_OTP "stm32f4xx-otp"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f4xx_OTP, STM32F4XX_OTP);

typedef struct Stm32f4xx_OTP {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    BlockBackend *blk;

    uint32_t data[OTP_SIZE];

} Stm32f4xx_OTP;

#endif //STM32F4XX_OTP_H
