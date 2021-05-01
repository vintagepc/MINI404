#include "qemu/osdep.h"
#include "qemu-common.h"
#include "stm32.h"
#include "hw/arm/armv7m.h"
#include "stm32_clktree.h"
#include "stm32_clk_type.h"

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
    Clk_t PERIPHCLK[STM32_PERIPH_COUNT]; \

/** RCC Base data structure */
// NOTE: the Clk entries cannot be pointers that are malloc'd like they originally were
// as this prevents you from being (easily) able to pass them through to the VMSTATE
// vmsd and take advantage of save states. Ergo, the array is fixed (no malloc) and every
// other area takes Clk_p pointers to a clock instead. 
struct Stm32Rcc {
    STM32_RCC_COMMON
};
