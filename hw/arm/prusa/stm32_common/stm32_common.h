#ifndef STM32_COMMON_H
#define STM32_COMMON_H

// Helpers for the components common between some/all STM32 family chips.

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "../utility/macros.h"
#include "stm32_shared.h"
#include "stm32_types.h"

typedef struct stm32_periph_variant_t {
	const char* variant_name;
	const stm32_reginfo_t* variant_regs;
} stm32_periph_variant_t;

typedef struct stm32_periph_banked_variant_t {
	const char* variant_name;
	const stm32_reginfo_t** variant_regs;
} stm32_periph_banked_variant_t;

// Definitions for the "base" peripheral type and its class/state:
// Note this isn't a clean opaque parent class but rather we are
// (ab)using the inheritance in a more tightly coupled way
// for convenience of member access in addition to reducing
// some of the generic boilerplate for the STM32 line.
typedef struct STM32Peripheral
{
	SysBusDevice parent;
	stm32_periph_t periph; // Contains the peripheral ID

	struct STM32COMRccState* rcc; // RCC interfacing with clocktree.

	/* DMA IRQ. To send a periperhal DMA Request, set the level to the peripheral's data register.
	* The DMA controller will check against its active streams for a match. and service it if one is found.
	*/
	qemu_irq dmar;

} STM32Peripheral;

typedef struct STM32PeripheralClass
{
	SysBusDeviceClass parent;
} STM32PeripheralClass;

// Common class data for variant storage.

// Base class for a SOC with a config blob.

typedef struct STM32SOCClass {
	SysBusDeviceClass parent_class;
    const struct stm32_soc_cfg_t* cfg; // Chip variant configuration store.
} STM32SOCClass;

typedef struct STM32SOC {
	SysBusDevice parent;
	char *cpu_type;
	hwaddr ram_size;
	hwaddr flash_size;
	DeviceState* perhiperhals[STM32_P_COUNT]; // Map for getting peripherals from chip variants.
} STM32SOC;


OBJECT_DECLARE_TYPE(STM32SOC, STM32SOCClass, STM32_SOC);

extern hwaddr stm32_soc_get_flash_size(DeviceState* soc);
extern hwaddr stm32_soc_get_sram_size(DeviceState* soc);
extern DeviceState* stm32_soc_get_periph(DeviceState* soc, stm32_periph_t id);


#endif //STM32_COMMON_H
