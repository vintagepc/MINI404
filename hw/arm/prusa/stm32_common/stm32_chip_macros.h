#ifndef STM32_CHIP_DEFS_H
#define STM32_CHIP_DEFS_H

#include "exec/hwaddr.h"
#include "stm32_shared.h"
#include "hw/core/split-irq.h"

enum {
    PERIPH_CFG_FLAG_NONE = 0,
    PERIPH_CFG_FLAG_NON_STM32P = 1,
};

// Add a single-line scalar entry with id and base address only.
#define PER_LN(id, typename, addr) [STM32_##id] = {typename, addr, 0, PERIPH_CFG_FLAG_NONE, {-1} }
// Add a single-line scalar entry with id, flags, and base address only.
#define PER_LNF(id, typename, addr, flags) [STM32_##id] = {typename, addr, 0, flags, {-1} }
// Add a single-line scalar entry with  id, base address, and IRQ vector
#define PER_LNI(id, typename, addr, ...) [STM32_##id] = {typename, addr, 0, PERIPH_CFG_FLAG_NONE, {__VA_ARGS__, -1}}
// Add a single-line scalar entry with  id, base address, flags, and IRQ vector
#define PER_LNIF(id, typename, addr, flags, ...) [STM32_##id] = {typename, addr, 0, flags, {__VA_ARGS__, -1}}

typedef struct stm32_periph_cfg_t
{
	const char* type;
	const hwaddr base_addr;
	const uint32_t size;
    const uint8_t flags;
	const int irq[17];
} stm32_periph_cfg_t;

// Struct for handling speicific variants' differing memory (flash/sram) sizes
typedef struct stm32_mem_cfg_t
{
	const char* chip_type;
	const hwaddr mem_size;
} stm32_mem_cfg_t;

// Global maxima for struct sizing
#define STM32_MAX_FLASH_OPTS 4 // maximum number of possible flash sizes for any one variant.

typedef struct stm32_soc_cfg_t
{
	const char* name;
	const uint32_t nvic_irqs;
	const uint32_t rcc_hsi_freq;
	const uint32_t rcc_hse_freq;
	const uint32_t rcc_lsi_freq;
	const uint32_t rcc_lse_freq;
	const hwaddr flash_base;
	const stm32_mem_cfg_t flash_variants[STM32_MAX_FLASH_OPTS +1];
	const hwaddr sram_base;
	const stm32_mem_cfg_t sram_variants[STM32_MAX_FLASH_OPTS +1];
	const hwaddr ccmsram_base;
	const stm32_mem_cfg_t ccmsram_variants[STM32_MAX_FLASH_OPTS +1];
	const stm32_periph_cfg_t perhipherals[STM32_P_COUNT];
} stm32_soc_cfg_t;

#endif
