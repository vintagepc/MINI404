#ifndef STM32_CHIP_DEFS_H
#define STM32_CHIP_DEFS_H

#include "exec/hwaddr.h"
#include "stm32_shared.h"
#include "hw/core/split-irq.h"

// Add a periph_t with an id, base address, and IRQ (no size)
#define PER_I(id,addr,irq) {STM32_##id, addr, 0, {irq, -1} }
// Add a periph_t with an id, base address, and IRQ vector (no size)
#define PER_IV(id,addr, ...) {STM32_##id, addr, 0, {__VA_ARGS__, -1}}
// ADd a periph_t with only ID and base address
#define PER(id,addr) {STM32_##id, addr, 0, {-1}}
// Add a single-line scalar entry with id and base address only.
#define PER_LN(name,typename, id, addr) .name = {.type = typename, .data = PER(id,addr)}
// Add a single-line scalar entry with  id, base address, and IRQ
#define PER_LNI(name, typename, id, addr, irq) .name = {.type = typename, .data = PER_I(id,addr,irq)}
// Add a single-line scalar entry with  id, base address, and IRQ vector
#define PER_LNIV(name, typename, id, addr, ...) .name = {.type = typename, .data = PER_IV(id,addr, __VA_ARGS__)}
// Add a single-line scalar entry with  id, base address, IRQ, and size.
#define PER_LNS(name, typename, id, addr, size) .name =  { .type = typename, .data = {STM32_##id, addr, size, {-1}} }
// Add a multiline entry where each additional arg beyond the typename is a PER() or PER_I()
#define PER_ALN(name, typename, ...) .name = {.type = typename, .data = { __VA_ARGS__ , PER(P_UNDEFINED, 0)}}

// Structure for specifying a peripheral's properties.
typedef struct stm32_periph_cfg_t
{
	const stm32_periph_t id; // the peripheral ID.
	const hwaddr base_addr; // Address in memory space at which it appears.
	const uint32_t size; // Size of the peripheral's memory mapping.
	const int irq[17];	// The IRQ array of the peripheral in the NVIC.
} stm32_periph_cfg_t;

// Struct for handling speicific variants' differing memory (flash/sram) sizes
typedef struct stm32_mem_cfg_t
{
	const char* chip_type;
	const hwaddr mem_size;
} stm32_mem_cfg_t;

// A simple single-data entry
#define PENTRY(name) const struct {const char* type; const stm32_periph_cfg_t data; } name;
// Multi-peripheral entry.
#define PENTRY_A(name,n) const struct {const char* type; const stm32_periph_cfg_t data[n+1]; } name;

// Convenience for scalar cfg-based init. Expects that Object* obj and cfg_t* cfg are local vars.
#define STM32_INIT(parent, fieldname) object_initialize_child(obj, #fieldname, &parent->fieldname, cfg->fieldname.type);

// Convenience for array cfg-based init. Expects that Object* obj and cfg_t* cfg are local vars.
#define STM32_INIT_A(parent,fieldname) \
{ \
	for (const stm32_periph_cfg_t* p = cfg->fieldname.data; p->id != STM32_P_UNDEFINED; p++ ){ \
		object_initialize_child(obj, #fieldname"[*]", &parent->fieldname[p - cfg->fieldname.data], cfg->fieldname.type); \
	} \
}

// Realize map, and connect IRQ for a peripheral &parent->f. Also connects RCC reset, if provided.
#define STM32_RLZ_AND_MAP(parent, f, cpu, set_periph, rcc) \
	if (cfg->f.data.id != STM32_P_UNDEFINED && STM32_SOC(parent) != NULL) \
	{ \
		STM32_SOC(parent)->perhiperhals[cfg->f.data.id] = DEVICE(&parent->f); \
	} \
	if (set_periph) \
	{ \
		QDEV_PROP_SET_PERIPH_T(DEVICE(&parent->f), "periph", cfg->f.data.id); \
	} \
	if (rcc != NULL) \
	{ \
		object_property_set_link(OBJECT(&parent->f),"rcc", OBJECT(rcc), &error_fatal); \
	} \
	if (!sysbus_realize(SYS_BUS_DEVICE(&parent->f), errp)) { \
		return; \
	} \
	sysbus_mmio_map(SYS_BUS_DEVICE(&parent->f), 0, cfg->f.data.base_addr); \
	for (const int *irq = cfg->f.data.irq; *irq != -1; irq++) \
	{ \
		sysbus_connect_irq(SYS_BUS_DEVICE(&parent->f), 0, qdev_get_gpio_in(cpu, *irq)); \
	} \
	if (rcc != NULL) \
	{ \
		qdev_connect_gpio_out_named(rcc,"reset",cfg->f.data.id,qdev_get_gpio_in_named(DEVICE(&parent->f),"rcc-reset",0)); \
	}


#define STM32_RLZ_AND_MAP_A(parent, f, n, cpu, rcc) \
	if (cfg->f.data[n].id != STM32_P_UNDEFINED && STM32_SOC(parent) != NULL) \
	{ \
		STM32_SOC(parent)->perhiperhals[cfg->f.data[n].id] = DEVICE(&parent->f[n]); \
	} \
	QDEV_PROP_SET_PERIPH_T(DEVICE(&parent->f[n]), "periph", cfg->f.data[n].id); \
	if (rcc != NULL) \
	{ \
		object_property_set_link(OBJECT(&parent->f[n]),"rcc", OBJECT(rcc), &error_fatal); \
	} \
	if (!sysbus_realize(SYS_BUS_DEVICE(&parent->f[n]), errp)) { \
		return; \
	} \
	sysbus_mmio_map(SYS_BUS_DEVICE(&parent->f[n]), 0, cfg->f.data[n].base_addr); \
	for (const int *irq = cfg->f.data[n].irq; *irq != -1; irq++) \
	{ \
		sysbus_connect_irq(SYS_BUS_DEVICE(&parent->f[n]), irq - cfg->f.data[n].irq, qdev_get_gpio_in(cpu, *irq)); \
	} \
	if (rcc != NULL) \
	{ \
		qdev_connect_gpio_out_named(rcc,"reset",cfg->f.data[n].id,qdev_get_gpio_in_named(DEVICE(&parent->f[n]),"rcc-reset",0)); \
	}

// Procsses the entire set of peripherals.
#define STM32_RLZ_MAP_ALL(parent, f, cpu, rcc) \
  	for (const stm32_periph_cfg_t* p = cfg->f.data; p->id != STM32_P_UNDEFINED; p++){ \
		STM32_RLZ_AND_MAP_A(parent, f, p - cfg->f.data, cpu, rcc); \
	}

// Connects the DMAR from a given peripheral to all chip DMA controllers.
#define STM32_CONNECT_DMAR(parent, p, dmas, n_dmas) \
	if (n_dmas == 1U) \
	{ \
		qdev_connect_gpio_out_named(DEVICE(&parent->p), "dmar", 0, qdev_get_gpio_in_named(DEVICE(&parent->dmas[0]),"dmar-in",0)); \
	} \
	else \
	{ \
		DeviceState *split_dmar = qdev_new(TYPE_SPLIT_IRQ); \
        qdev_prop_set_uint16(split_dmar, "num-lines", n_dmas); \
		qdev_realize_and_unref(split_dmar, NULL,  &error_fatal); \
		qdev_connect_gpio_out_named(DEVICE(&parent->p), "dmar", 0, qdev_get_gpio_in(split_dmar, 0)); \
		for (int i=0; i<n_dmas; i++) \
		{ \
			qdev_connect_gpio_out(split_dmar, i,  qdev_get_gpio_in_named(DEVICE(&parent->dmas[i]),"dmar-in",0)); \
		} \
	}

// Connects the DMARs from a given peripheral array to all chip DMA controllers.
#define STM32_CONNECT_DMAR_A(parent, f, ...) \
  	for (const stm32_periph_cfg_t* p = cfg->f.data; p->id != STM32_P_UNDEFINED; p++){ \
		STM32_CONNECT_DMAR(parent, f[p - cfg->f.data], __VA_ARGS__ ); \
	}


// Global maxima for struct sizing
#define STM32_MAX_ADCS 3
#define STM32_MAX_UARTS 8
#define STM32_MAX_DMAS 2
#define STM32_MAX_I2CS 6
#define STM32_MAX_SPIS 6
#define STM32_MAX_GPIOS 11
#define STM32_MAX_TIMERS 17
#define STM32_MAX_FLASH_OPTS 4 // maximum number of possible flash sizes for any one variant.

typedef struct stm32_soc_cfg_t
{
	const uint32_t nvic_irqs;
	const uint32_t rcc_hsi_freq;
	const uint32_t rcc_hse_freq;
	const uint32_t rcc_lsi_freq;
	const uint32_t rcc_lse_freq;
	const stm32_mem_cfg_t flash_variants[STM32_MAX_FLASH_OPTS +1];
	const stm32_mem_cfg_t sram_variants[STM32_MAX_FLASH_OPTS +1];
	PENTRY(flash_memory);
	PENTRY(sram);
	PENTRY(ccmsram);
	PENTRY(syscfg);
	PENTRY_A(usarts, STM32_MAX_UARTS);
	PENTRY_A(adcs, STM32_MAX_ADCS);
	PENTRY_A(spis, STM32_MAX_I2CS);
	PENTRY_A(i2cs, STM32_MAX_SPIS);
	PENTRY_A(dmas, STM32_MAX_DMAS);
	PENTRY_A(gpios, STM32_MAX_GPIOS);
	PENTRY_A(timers, STM32_MAX_TIMERS);
	PENTRY(exti);
	PENTRY(eth);
	PENTRY(rcc);
	PENTRY(flash_if);
	PENTRY(iwdg);
	PENTRY(crc);
	PENTRY(rtc);
	PENTRY(itm);
	PENTRY(dwt);
	PENTRY(pwr);
	PENTRY(rng);
	PENTRY(otp);
	PENTRY(usb_hs);
	PENTRY(usb_fs);
	PENTRY(adc_common);
} stm32_soc_cfg_t;

#endif
