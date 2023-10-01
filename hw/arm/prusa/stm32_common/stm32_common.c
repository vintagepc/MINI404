#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "sysemu/block-backend.h"
#include "hw/boards.h"
#include "hw/qdev-core.h"
#include "qapi/error.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/qdev-properties.h"
#include "stm32_common.h"
#include "stm32_chip_macros.h"
#include "stm32_rcc.h"
#include "stm32_rcc_if.h"
#include "stm32_shared.h"

DECLARE_CLASS_CHECKERS(STM32PeripheralClass, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

DECLARE_INSTANCE_CHECKER(STM32Peripheral, STM32_PERIPHERAL, TYPE_STM32_PERIPHERAL);

static bool create_if_not_exist(const char* default_name, uint32_t file_size)
{
	bool exists = true;
	if (access(default_name, R_OK | W_OK) == -1)
	{
		printf("%s not found - creating it.\n",default_name);
		// Create it.
		int fd = creat(default_name, S_IRUSR | S_IWUSR);
		exists = (ftruncate(fd, file_size) != -1);
		close(fd);
	}
	return exists;
}

extern BlockBackend* get_or_create_drive(BlockInterfaceType interface, int index, const char* default_name, const char* label, uint32_t file_size, Error** errp)
{
	BlockBackend *blk = blk_by_name(label);
	if (blk)
	{
		return blk;
	}
	DriveInfo* dinfo = drive_get(interface, 0, index);
	if (!dinfo)
	{
		if (create_if_not_exist(default_name, file_size))
		{
			printf("No -%s drive specified, using default %s\n",
				interface==IF_MTD? "mtdblock" : "pflash",
				default_name);
			QemuOpts* drive_opts = drive_add(interface, index, default_name, "format=raw");
			dinfo = drive_new(drive_opts, interface, errp);
		}
	}
	return blk_by_legacy_dinfo(dinfo);
}

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

extern void stm32_soc_setup_flash(DeviceState* dev, MemoryRegion* flash, Error** errp)
{
	STM32SOC* soc = STM32_SOC(dev);
	STM32SOCClass *class = STM32_SOC_GET_CLASS(soc);
	hwaddr flash_size = stm32_soc_get_flash_size(dev);
	assert(class->cfg->name);

	gchar* flash_name = g_strdup_printf("%s.flash",class->cfg->name);
	// TODO - figure out mmap for msys2 sometime...
#ifdef CONFIG_POSIX
	if (soc->flash_filename != NULL)
	{
		create_if_not_exist(soc->flash_filename, flash_size);

		soc->flash_fd = open(soc->flash_filename , O_CREAT| O_RDWR | O_CLOEXEC, 0644 );

		void *mem = mmap(NULL, flash_size, PROT_READ | PROT_WRITE, MAP_SHARED, soc->flash_fd, 0);
		if (mem == MAP_FAILED)
		{
			fprintf(stderr, "Failed to mmap flash file %s: %d\n", soc->flash_filename, errno);
			close(soc->flash_fd);
		}
		else
		{
			uint8_t* p_mem = (uint8_t*)mem;
			bool is_blank = true;
			for (int i=0; i<flash_size; i++)
			{
				if (p_mem[i]!=0)
				{
					is_blank = false;
					break;
				}
			}
			printf("Using file-backed flash storage for %s: %s\n", class->cfg->name, soc->flash_filename);
			if (is_blank)
			{
				printf("Backing file is all null, filling with 0xFF\n");
				memset(mem, 0xFF, flash_size);
			}
			memory_region_init_ram_ptr(flash, OBJECT(soc), flash_name, flash_size, mem);
			memory_region_set_readonly(flash, true);
			return; // Done here. don't execute the non-file backed setup.
		}
	}
#endif
	printf("# No flash filename configured, error opening file, or mmap is not supported (W64). Skipping file-backed flash.\n");
	// Executed on error or if no filename given.
	memory_region_init_rom(flash, OBJECT(soc), flash_name,
                           flash_size, errp);

	g_free(flash_name);
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
	for (const stm32_mem_cfg_t* p = cfg->ccmsram_variants; p->chip_type != NULL; p++){
		if (strcmp(type, p->chip_type) == 0)
		{
			return p->mem_size;
		}
	}
	printf("ERR: CCMSRAM size not defined in variant array for %s!\n", type);
	g_assert_not_reached();
}

extern bool stm32_soc_is_periph(DeviceState* soc, stm32_periph_t id)
{
	STM32SOC *s = STM32_SOC(soc);
	return (s->perhiperhals[id] != NULL);

}

extern DeviceState* stm32_soc_get_periph(DeviceState* soc, stm32_periph_t id)
{
	STM32SOC *s = STM32_SOC(soc);
	if (s->perhiperhals[id] == NULL)
	{
		printf("# ERR: Asked to retreive a peripheral (%s) that's not defined by the SOC!\n", _PERIPHNAMES[id]);
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

stm32_periph_t g_stm32_periph_init = STM32_P_UNDEFINED;

static void stm32_soc_instance_init(Object* obj)
{
	STM32SOCClass *c = STM32_SOC_GET_CLASS(obj);
	STM32SOC *s = STM32_SOC(obj);
	for (int i=0; i<STM32_P_COUNT; i++)
	{
		const stm32_periph_cfg_t* p = &(c->cfg->perhipherals[i]);
		if (p->type != NULL)
		{
			g_stm32_periph_init = i;
			s->perhiperhals[i] = qdev_new(p->type);
			object_property_add_child(obj, _PERIPHNAMES[i], OBJECT(s->perhiperhals[i]));
			g_stm32_periph_init = STM32_P_UNDEFINED;
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
	for (uint8_t dmar_t = DMAR_TYPE_BEGIN; dmar_t < DMAR_TYPE_END; dmar_t++)
	{
		if (n_dmas == 1U)
		{
			qdev_connect_gpio_out_named(s->perhiperhals[id], "dmar", dmar_t, qdev_get_gpio_in_named(s->perhiperhals[STM32_P_DMA1],"dmar-in",dmar_t)); \
		}
		else
		{
			DeviceState *split_dmar = qdev_new(TYPE_SPLIT_IRQ);
			qdev_prop_set_uint16(split_dmar, "num-lines", n_dmas);
			qdev_realize_and_unref(split_dmar, NULL,  errp);
			qdev_connect_gpio_out_named(s->perhiperhals[id], "dmar", dmar_t, qdev_get_gpio_in(split_dmar, 0));
			for (int i=STM32_P_DMA_BEGIN; i<= STM32_P_DMA_END; i++)
			{
				qdev_connect_gpio_out(split_dmar, i-STM32_P_DMA_BEGIN,  qdev_get_gpio_in_named(s->perhiperhals[i],"dmar-in",dmar_t));
			}
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
			qemu_log_mask(LOG_UNIMP, "Warning: Peripheral %s does not support the STM32 Common interface\n", _PERIPHNAMES[id]);
		}
	}
	if (!sysbus_realize(SYS_BUS_DEVICE(s->perhiperhals[id]), errp)) {
		return;
	}
	// ITM is special and can't go in the sysbus region. see the stm32f4xx_soc.c for more.
	if (id !=STM32_P_ITM && id != STM32_P_DWT)
	{
		if (s->has_sys_memory)
		{
			SysBusDevice*d = SYS_BUS_DEVICE(s->perhiperhals[id]);
			// Decorate the MMIO names now that we know the actual peripheral ID.
			d->mmio->memory->name = _PERIPHNAMES[id];
			memory_region_add_subregion(&s->sys_memory, cfg->base_addr, d->mmio[0].memory);
		}
		else
		{
		 	sysbus_mmio_map(SYS_BUS_DEVICE(s->perhiperhals[id]), 0, cfg->base_addr);
		}
	}
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
    qdev_init_gpio_out_named(DEVICE(obj),s->dmar,"dmar",2);
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
	DEFINE_PROP_STRING("flash-file", STM32SOC, flash_filename),
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

extern void stm32_soc_machine_init(MachineState *machine)
{
	DeviceState *dev;
	STM32SocMachineClass *smc = STM32_MACHINE_GET_CLASS(OBJECT(machine));

    dev = qdev_new(smc->soc_type);
	object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_prop_set_string(dev, "cpu-type", smc->cpu_type);
    qdev_prop_set_uint32(dev,"sram-size", machine->ram_size);
	uint64_t flash_size = stm32_soc_get_flash_size(dev);
	sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);

	armv7m_load_kernel(ARM_CPU(first_cpu),
					machine->kernel_filename,
					0, flash_size);
}

qemu_irq qemu_irq_split(qemu_irq irq1, qemu_irq irq2)
{
    DeviceState* splitter = qdev_new(TYPE_SPLIT_IRQ);
    qdev_prop_set_uint32(splitter, "num-lines", 2);
    qdev_realize_and_unref(splitter, NULL, &error_fatal);
    qdev_connect_gpio_out(splitter, 0, irq1);
    qdev_connect_gpio_out(splitter, 1, irq2);
    return qdev_get_gpio_in(splitter, 0);
}
