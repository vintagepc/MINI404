#include "qemu/osdep.h"
#include "qom/object.h"
#include "qemu-common.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "stm32_common.h"
#include "stm32_chip_macros.h"
#include "stm32_rcc.h"
#include "stm32_rcc_if.h"

DECLARE_CLASS_CHECKERS(STM32PeripheralClass, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

DECLARE_INSTANCE_CHECKER(STM32Peripheral, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

extern hwaddr stm32_soc_get_flash_size(DeviceState* dev)
{
	STM32SOC* soc = STM32_SOC(dev);
	if (soc->flash_size == 0)
	{
		const stm32_soc_cfg_t* cfg = STM32_SOC_GET_CLASS(dev)->cfg;
		const char* type = object_get_typename(OBJECT(dev));
		for (const stm32_mem_cfg_t* p = cfg->flash_variants; p->chip_type != NULL; p++){
			if (strcmp(type, p->chip_type) == 0)
			{
				return p->mem_size;
			}
		}
		printf("ERR: Flash size not defined in variant array for %s!\n", type);
		return cfg->flash_memory.data.size;
	}
	else
		return soc->flash_size;
}

extern hwaddr stm32_soc_get_sram_size(DeviceState* dev)
{
	STM32SOC* soc = STM32_SOC(dev);
	if (soc->ram_size == 0)
	{
		const stm32_soc_cfg_t* cfg = STM32_SOC_GET_CLASS(dev)->cfg;
		const char* type = object_get_typename(OBJECT(dev));
		for (const stm32_mem_cfg_t* p = cfg->sram_variants; p->chip_type != NULL; p++){
			if (strcmp(type, p->chip_type) == 0)
			{
				return p->mem_size;
			}
		} // If no variants, just return the fixed value.
		return cfg->sram.data.size;
	}
	else
		return soc->ram_size;
}

extern DeviceState* stm32_soc_get_periph(DeviceState* soc, stm32_periph_t id)
{
	STM32SOC *s = STM32_SOC(soc);
	if (s->perhiperhals[id] == NULL)
	{
		printf("ERR: Asked to retreive a peripheral that's not defined by the SOC!\n");
	}
	return s->perhiperhals[id];
}


static void stm32_peripheral_rcc_reset(void *opaque, int n, int level)
{
	if (level)
	{
		DeviceClass *c = DEVICE_GET_CLASS(opaque);
		c->reset(DEVICE(opaque));
	}
}

static void stm32_peripheral_instance_init(Object* obj)
{
	qdev_init_gpio_in_named(DEVICE(obj),  stm32_peripheral_rcc_reset, "rcc-reset",1);
	STM32Peripheral *s = STM32_PERIPHERAL(obj);
    qdev_init_gpio_out_named(DEVICE(obj),&s->dmar,"dmar",1);

}

static Property stm32_peripheral_properties[] = {
	DEFINE_PROP_PERIPH_T("periph", STM32Peripheral, periph, -1),
	DEFINE_PROP_LINK("rcc", STM32Peripheral, rcc, TYPE_STM32COM_RCC_IF, COM_STRUCT_NAME(Rcc)*),
	DEFINE_PROP_END_OF_LIST()
};

static void stm32_peripheral_class_init(ObjectClass* class, void* class_data)
{
	DeviceClass *dc = DEVICE_CLASS(class);
	device_class_set_props(dc, stm32_peripheral_properties);
}

static const TypeInfo stm32_peripheral_info = {
	.name = TYPE_STM32_PERIPHERAL,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(STM32Peripheral),
	.instance_init = stm32_peripheral_instance_init,
	.abstract = true,
	.class_size = sizeof(STM32PeripheralClass),
	.class_init = stm32_peripheral_class_init,
};

static Property stm32_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", 	STM32SOC, cpu_type),
    DEFINE_PROP_UINT64("sram-size",	STM32SOC, ram_size, 0), // 0 = use chip default
	DEFINE_PROP_UINT64("flash-size",STM32SOC, flash_size, 0), //
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32_soc_class_init(ObjectClass* class, void* class_data)
{
	DeviceClass *dc = DEVICE_CLASS(class);
	device_class_set_props(dc, stm32_soc_properties);
}

static const TypeInfo stm32_soc_info = {
	.name = TYPE_STM32_SOC,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(STM32SOC),
	.abstract = true,
	.class_size = sizeof(STM32SOCClass),
	.class_init = stm32_soc_class_init,
};

static void stm32_base_register_types(void)
{
	type_register_static(&stm32_peripheral_info);
	type_register_static(&stm32_soc_info);
}

type_init(stm32_base_register_types);
