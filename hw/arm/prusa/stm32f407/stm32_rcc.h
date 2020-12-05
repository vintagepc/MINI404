#include "qemu/osdep.h"
#include "qemu-common.h"
#include "stm32.h"
#include "hw/arm/armv7m.h"
#include "stm32_clktree.h"


#define STM32_RCC_COMMON \
    SysBusDevice busdev; \
 \
    /* Properties */ \
    uint32_t osc_freq; \
    uint32_t osc32_freq; \
 \
    /* Private */ \
    MemoryRegion iomem; \
    qemu_irq irq; \
 \
    /* Peripheral clocks */ \
    Clk PERIPHCLK[STM32_PERIPH_COUNT]; \

/** RCC Base data structure */
struct Stm32Rcc {
    STM32_RCC_COMMON
};
