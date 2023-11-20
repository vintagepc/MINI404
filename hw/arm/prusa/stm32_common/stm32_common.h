#ifndef STM32_COMMON_H
#define STM32_COMMON_H

// Helpers for the components common between some/all STM32 family chips.

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "sysemu/blockdev.h"
#include "hw/boards.h"
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

    // The clock frequency as output from the RCC
    uint32_t clock_freq;
    bool clock_enabled;
	/* DMA IRQ. To send a periperhal DMA Request, set the level to the peripheral's data register.
	* The DMA controller will check against its active streams for a match. and service it if one is found.
	* Use index 1 if this is meant to be a write (M2P) or "ready for data" case - some peripherals have
	* shared RDR/TDRs which we must be able to distinguish in order to service things in the right order...
	*/
	qemu_irq dmar[2];

} STM32Peripheral;

DECLARE_INSTANCE_CHECKER(STM32Peripheral, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

enum DMAR_TYPE {
	DMAR_TYPE_BEGIN,
	DMAR_P2M = 0, // Read operation, i.e. DMA takes data from the device.
	DMAR_M2P, // DMA controller asked to supply data to device, if it has any.
	DMAR_TYPE_END
};

enum EXTI_TRANS {
	EXTI_RISING  = 0b01, // 0->1 transition (rising edge)
	EXTI_FALLING = 0b10 // 1->0 transition
};

typedef struct STM32PeripheralClass
{
	SysBusDeviceClass parent;
    // A handler to set a callback if the input clock changes.
    void (*clock_update)(STM32Peripheral *p);
} STM32PeripheralClass;

DECLARE_CLASS_CHECKERS(STM32PeripheralClass, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

// Common class data for variant storage.

// Machine class templates:
typedef struct STM32SocMachineClass {
    MachineClass        parent;

    const char          *soc_type;
    const char          *cpu_type;
} STM32SocMachineClass;

#define STM32_MACHINE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(STM32SocMachineClass, (klass), TYPE_STM32_MACHINE)
#define STM32_MACHINE_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(STM32SocMachineClass, (obj), TYPE_STM32_MACHINE)

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
	DeviceState *cpu;
	char* flash_filename; //default file-backed storage for flash.
	int flash_fd;
	MemoryRegion sys_memory; // Local CPU system memory. We need this for our multi-cpu instances...
	AddressSpace as_sys_memory;
	bool has_sys_memory;
} STM32SOC;

OBJECT_DECLARE_TYPE(STM32SOC, STM32SOCClass, STM32_SOC);

extern void stm32_soc_machine_init(MachineState *machine);

extern hwaddr stm32_soc_get_flash_size(DeviceState* soc);
extern hwaddr stm32_soc_get_sram_size(DeviceState* soc);
extern hwaddr stm32_soc_get_ccmsram_size(DeviceState* soc);
extern bool stm32_soc_is_periph(DeviceState* soc, stm32_periph_t id);
extern DeviceState* stm32_soc_get_periph(DeviceState* soc, stm32_periph_t id);
extern void stm32_soc_realize_peripheral(DeviceState* soc_state, stm32_periph_t id, Error **errp);
extern void stm32_soc_realize_all_peripherals(DeviceState *soc_state,Error **errp);

extern void stm32_soc_setup_flash(DeviceState* soc, MemoryRegion* flash, Error** errp);

extern BlockBackend* get_or_create_drive(BlockInterfaceType interface, int index, const char* default_name, const char* label, uint32_t file_size, Error** errp);


#endif //STM32_COMMON_H
