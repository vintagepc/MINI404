#include "qemu/osdep.h"
#include "qom/object.h"
#include "qemu-common.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "stm32_common.h"
#include "stm32_chip_macros.h"
#include "stm32_rcc.h"
#include "stm32_rcc_if.h"
#include "stm32_shared.h"

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
		g_assert_not_reached();
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
		}
		printf("ERR: SRAM size not defined in variant array for %s!\n", type);
		g_assert_not_reached();
	}
	else
		return soc->ram_size;
}
extern hwaddr stm32_soc_get_ccmsram_size(DeviceState* dev)
{
	const stm32_soc_cfg_t* cfg = STM32_SOC_GET_CLASS(dev)->cfg;
	const char* type = object_get_typename(OBJECT(dev));
	for (const stm32_mem_cfg_t* p = cfg->ccmssram_variants; p->chip_type != NULL; p++){
		if (strcmp(type, p->chip_type) == 0)
		{
			return p->mem_size;
		}
	}
	printf("ERR: CCMSRAM size not defined in variant array for %s!\n", type);
	g_assert_not_reached();
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

static void stm32_soc_instance_init(Object* obj)
{
	STM32SOCClass *c = STM32_SOC_GET_CLASS(obj);
	STM32SOC *s = STM32_SOC(obj);
	for (int i=0; i<STM32_P_COUNT; i++)
	{
		const stm32_periph_cfg_t* p = &(c->cfg->perhipherals[i]);
		if (p->type != NULL)
		{
			s->perhiperhals[i] = qdev_new(p->type);
			object_property_add_child(obj, _PERIPHNAMES[i], OBJECT(s->perhiperhals[i]));
		}
	}
}

static void stm32_soc_connect_periph_dmar(DeviceState* soc_state, stm32_periph_t id, uint8_t n_dmas, Error **errp)
{
	STM32SOC *s = STM32_SOC(soc_state);
	if (s->perhiperhals[id] == NULL || !object_dynamic_cast(OBJECT(s->perhiperhals[id]), TYPE_STM32_PERIPHERAL))
	{
		return;
	}
	if (n_dmas == 1U)
	{
		qdev_connect_gpio_out_named(s->perhiperhals[id], "dmar", 0, qdev_get_gpio_in_named(s->perhiperhals[STM32_P_DMA1],"dmar-in",0)); \
	}
	else
	{
		DeviceState *split_dmar = qdev_new(TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(split_dmar, "num-lines", n_dmas);
		qdev_realize_and_unref(split_dmar, NULL,  errp);
		qdev_connect_gpio_out_named(s->perhiperhals[id], "dmar", 0, qdev_get_gpio_in(split_dmar, 0));
		for (int i=STM32_P_DMA_BEGIN; i<= STM32_P_DMA_END; i++)
		{
			qdev_connect_gpio_out(split_dmar, i-STM32_P_DMA_BEGIN,  qdev_get_gpio_in_named(s->perhiperhals[i],"dmar-in",0));
		}
	}
}

extern void stm32_soc_realize_peripheral(DeviceState* soc_state, stm32_periph_t id, Error **errp)
{
	STM32SOCClass *c = STM32_SOC_GET_CLASS(soc_state);
	const stm32_periph_cfg_t *cfg = &(c->cfg->perhipherals[id]);
	STM32SOC *s = STM32_SOC(soc_state);
	if (s == NULL || s->perhiperhals[id] == NULL || cfg->type == NULL)
	{
		return;
	}
	if (id>STM32_P_RCC && s->perhiperhals[STM32_P_RCC] != NULL)
	{
		if (object_property_find(OBJECT(s->perhiperhals[id]), "periph"))
		{
			QDEV_PROP_SET_PERIPH_T(s->perhiperhals[id], "periph", id);
			object_property_set_link(OBJECT(s->perhiperhals[id]),"rcc", OBJECT(s->perhiperhals[STM32_P_RCC]), errp);
			qdev_connect_gpio_out_named(s->perhiperhals[STM32_P_RCC],"reset",id,qdev_get_gpio_in_named(s->perhiperhals[id],"rcc-reset",0));
		}
		else
		{
			printf("Warning: Peripheral %s does not support the STM32 Common interface\n", _PERIPHNAMES[id]);
		}
	}
	if (!sysbus_realize(SYS_BUS_DEVICE(s->perhiperhals[id]), errp)) {
		return;
	}
	// ITM is special and can't go in the sysbus region. see the stm32f4xx_soc.c for more.
	if (id !=STM32_P_ITM) sysbus_mmio_map(SYS_BUS_DEVICE(s->perhiperhals[id]), 0, cfg->base_addr);
	for (const int *irq = cfg->irq; *irq != -1; irq++)
	{
		sysbus_connect_irq(SYS_BUS_DEVICE(s->perhiperhals[id]), irq-(cfg->irq), qdev_get_gpio_in(s->cpu, *irq));
	}
}

extern void stm32_soc_realize_all_peripherals(DeviceState *soc_state,Error **errp)
{
	STM32SOCClass *c = STM32_SOC_GET_CLASS(soc_state);
	uint8_t n_dmas = 0;
	for (int i=STM32_P_DMA_BEGIN; i<= STM32_P_DMA_END; i++)
	{
		if (!c->cfg->perhipherals[i].type) break;
		n_dmas++;
	}
	for (int i=0; i<STM32_P_COUNT; i++)
	{
		stm32_soc_realize_peripheral(soc_state, i, errp);
		// Auto wire the DMAR.
		stm32_soc_connect_periph_dmar(soc_state, i, n_dmas,errp);
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
	.instance_init = stm32_soc_instance_init,
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
