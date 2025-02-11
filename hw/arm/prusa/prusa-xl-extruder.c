/*
 * Prusa Buddy board machine model
 *
 * Copyright 2020 VintagePC <github.com/vintagepc>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/xl_bridge.h"

#define BOOTLOADER_IMAGE "bl_dwarf.elf.bin"
#define TYPE_XLEXTRUDER_MACHINE "xlextruder-machine"

enum HW_VER
{
	E_STM32G0,
	E_STM32G0_0_4_0,
	E_HW_VER_COUNT,
};

#define NUM_THERM_MAX 3

typedef struct prusa_xl_e_cfg_t
{
	// Hotend, board, heatbreak
	uint8_t therm_channels[NUM_THERM_MAX];
	uint16_t therm_tables[NUM_THERM_MAX];
	bool invert_e0_dir;
} prusa_xl_e_cfg_t;

typedef struct xlExtruderData {
    uint8_t hw_type;
    uint8_t tool_index;
	const char* descr;
    const char* flash_filename;
    const char* tool_name;
} xlExtruderData;

typedef struct xlExtruderMachineClass {
    MachineClass parent_class;
    const char* flash_filename;
    const char* tool_name;
    uint8_t hw_type;
    uint8_t tool_index;
} xlExtruderMachineClass;

#define ADD_CFG(hwtype, index) \
    static const xlExtruderData xl_extruder_data_##hwtype##index = { \
        .hw_type = hwtype, \
        .tool_index = index, \
        .descr = "Prusa XL Extruder Board Tool " #index, \
        .flash_filename = "Prusa_XL_Dwarf_" #index "_flash.bin", \
        .tool_name = "Dwarf " #index \
    };

#define ADD_VER_CFGS(hwtype) \
    ADD_CFG(hwtype, 0); \
    ADD_CFG(hwtype, 1); \
    ADD_CFG(hwtype, 2); \
    ADD_CFG(hwtype, 3); \
    ADD_CFG(hwtype, 4);

ADD_VER_CFGS(E_STM32G0);
ADD_VER_CFGS(E_STM32G0_0_4_0);

#undef ADD_CFG
#undef ADD_VER_CFGS

// F0 and first revision of G0
static const prusa_xl_e_cfg_t extruder_g0 = {
	.therm_channels = {5, 7, 10},
	.therm_tables = {2005, 2000, 2008},
	.invert_e0_dir = false,
};

// Temp is off by a few degrees for the hotend, I think the temp table here is not right - but at least it no longer MAXTEMPs right now.
static const prusa_xl_e_cfg_t extruder_g0_v0_4_0 = {
	.therm_channels = {10, 7, 5}, // HB/Nozzle swapped positions and tables changed.
	.therm_tables = {2007, 2000, 5},
	.invert_e0_dir = true,
};

static const prusa_xl_e_cfg_t* extruder_cfg_map[E_HW_VER_COUNT] =
{
	[E_STM32G0] = &extruder_g0,
	[E_STM32G0_0_4_0] = &extruder_g0_v0_4_0
};

#define XLEXTRUDER_MACHINE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(xlExtruderMachineClass, (klass), TYPE_XLEXTRUDER_MACHINE)
#define XLBUDDY_MACHINE_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(xlExtruderMachineClass, (obj), TYPE_XLEXTRUDER_MACHINE)

static void prusa_xl_extruder_init(MachineState *machine)
{
    DeviceState *dev;
    const xlExtruderMachineClass *mc = XLBUDDY_MACHINE_GET_CLASS(OBJECT(machine));
    Object* periphs = container_get(OBJECT(machine), "/peripheral");

	const prusa_xl_e_cfg_t* cfg = extruder_cfg_map[mc->hw_type];

	dev = qdev_new(TYPE_STM32G070xB_SOC);

	// TODO.. can we somehow detect if an extruder is already running and auto-increment the index?
	// maybe with flock on the extuder flash filename?

	DeviceState* dev_soc = dev;
	qdev_prop_set_string(dev, "flash-file", mc->flash_filename);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.

    char* kfn = machine->kernel_filename;
    int kernel_len = kfn ? strlen(kfn) : 0;
    if (kernel_len > 0) arghelper_setargs(machine->kernel_cmdline);
    if (kernel_len >3 && strncmp(kfn + (kernel_len-3), "bbf",3) == 0 )
    {
        // TODO... use initrd_image as a bootloader alternative?
        struct stat bootloader;
        if (stat(BOOTLOADER_IMAGE,&bootloader))
        {
            error_setg(&error_fatal, "No %s file found. It is required to use a .bbf file!",BOOTLOADER_IMAGE);
        }
        // BBF has an extra 64b header we need to prune. Rather than modify it or use a temp file, offset it
        // by -64 bytes and rely on the bootloader clobbering it.
        stm32_soc_load_targphys(OBJECT(dev_soc), machine->kernel_filename, 0x08000000);
        stm32_soc_load_kernel(OBJECT(dev_soc), BOOTLOADER_IMAGE);
    }
    else if (kernel_len > 0) // Raw bin or ELF file, load directly.
    {
        stm32_soc_load_kernel(OBJECT(dev_soc), machine->kernel_filename);
    }

	DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

	DeviceState* dashboard = qdev_new("2d-dashboard");
    qdev_prop_set_uint8(dashboard, "fans", 2);
    qdev_prop_set_uint8(dashboard, "thermistors", 3);
    qdev_prop_set_string(dashboard, "indicators", "LPFDC");
    qdev_prop_set_string(dashboard, "title", mc->tool_name);

	DeviceState* motor = NULL;

	motor = qdev_new("tmc2130");

	qdev_prop_set_uint8(motor, "axis",'E');
	qdev_prop_set_uint8(motor, "inverted", cfg->invert_e0_dir);
	qdev_prop_set_int32(motor, "max_step", 0);
	qdev_prop_set_int32(motor, "fullstepspermm", 380);

	DeviceState* p = qdev_new("software-spi");
	sysbus_realize(SYS_BUS_DEVICE(p),  &error_fatal);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 6,qdev_get_gpio_in_named(p,"mosi",0));
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB), 3,qdev_get_gpio_in_named(p,"sck",0));
	qdev_connect_gpio_out_named(p,"miso",0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD),5));

	qdev_realize(motor, qdev_get_child_bus(p,"ssi"), &error_fatal);
	qemu_irq driver_cs = qdev_get_gpio_in_named(motor, SSI_GPIO_CS, 0);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB), 4, driver_cs);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 2,qdev_get_gpio_in_named(motor,"enable",0));
	qdev_connect_gpio_out_named(motor,"diag", 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD),3));
	qdev_connect_gpio_out_named(motor,"spi-peek", 0, qdev_get_gpio_in_named(p, "miso-byte",0));


	object_property_set_link(OBJECT(dashboard), "motor[0]", OBJECT(motor), &error_fatal);


	sysbus_realize(SYS_BUS_DEVICE(dashboard), &error_fatal);

	// qdev_connect_gpio_out_named(vis, "pick",0,qdev_get_gpio_in(DEVICE(&SOC->exti),4));
	// qdev_connect_gpio_out_named(vis, "pick",0,qdev_get_gpio_in(DEVICE(&SOC->exti),5));
	// qdev_connect_gpio_out_named(vis, "pick",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),4));
	// qdev_connect_gpio_out_named(vis, "pick",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),5));

	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),11,qdev_get_gpio_in_named(dashboard, "led-digital",1));
	dev = qdev_new("dwarf-input");
    object_property_add_child(periphs, "dwarf-input", OBJECT(dev));
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
	qdev_connect_gpio_out(dev,0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),15));
	qdev_connect_gpio_out(dev,1,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),10));

	dev = qdev_new("ws281x");
    object_property_add_child(periphs, "ws281x", OBJECT(dev));
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB),6,qdev_get_gpio_in(dev,0));
	qdev_connect_gpio_out_named(dev,"colour",0,qdev_get_gpio_in_named(dashboard, "led-rgb",0));

	DeviceState* hall = qdev_new("hall-sensor");
	qdev_prop_set_uint32(hall,"present-value",70);
	qdev_prop_set_uint32(hall,"missing-value",3000);
	sysbus_realize(SYS_BUS_DEVICE(hall), &error_fatal);
	qdev_connect_gpio_out_named(hall,"status", 0, qdev_get_gpio_in_named(dashboard,"led-digital",2));
	qdev_connect_gpio_out(hall, 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in", 4));

	DeviceState* park_sense = qdev_new("hall-sensor");
	qdev_prop_set_uint32(park_sense,"present-value", 1400);
	qdev_prop_set_uint32(park_sense,"missing-value", 770);
	sysbus_realize(SYS_BUS_DEVICE(park_sense), &error_fatal);
	qdev_connect_gpio_out_named(park_sense,"status", 0, qdev_get_gpio_in_named(dashboard,"led-digital",3));
	qdev_connect_gpio_out(park_sense, 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in", 2));

    DeviceState* pick_sense = qdev_new("hall-sensor");
	qdev_prop_set_uint32(pick_sense,"present-value", 1400);
	qdev_prop_set_uint32(pick_sense,"missing-value", 770);
    qdev_prop_set_bit(pick_sense, "start-state", 0);
	sysbus_realize(SYS_BUS_DEVICE(pick_sense), &error_fatal);
	qdev_connect_gpio_out_named(pick_sense,"status", 0, qdev_get_gpio_in_named(dashboard,"led-digital",4));
	qdev_connect_gpio_out(pick_sense, 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in", 1));


    //0 is HBR
    //1 is Print fan
    uint16_t fan_max_rpms[] = { 8000, 6000 };
    uint8_t  fan_pwm_pins[] = { 7, 6};
    uint8_t fan_tach_exti_lines[] = { 8, 9};
    uint8_t fan_labels[] = {'P','E'};
	DeviceState* fanpwm = qdev_new("software-pwm");
	sysbus_realize_and_unref(SYS_BUS_DEVICE(fanpwm),&error_fatal);
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM14), "timer", 0, qdev_get_gpio_in_named(fanpwm, "tick-in", 0));
    for (int i=0; i<2; i++)
    {
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC), fan_pwm_pins[i],
			qdev_get_gpio_in_named(fanpwm, "gpio-in",i)
		);
        dev = qdev_new("fan");
        qdev_prop_set_uint8(dev,"label",fan_labels[i]);
        qdev_prop_set_uint32(dev, "max_rpm",fan_max_rpms[i]);
        qdev_prop_set_bit(dev, "is_nonlinear", i); // E is nonlinear.
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(dev, "tach-out",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),fan_tach_exti_lines[i]));
		qemu_irq split_fan = qemu_irq_split( qdev_get_gpio_in_named(dev, "pwm-in",0), qdev_get_gpio_in_named(dashboard, "fan-pwm",i));
		qdev_connect_gpio_out_named(dev, "rpm-out", 0, qdev_get_gpio_in_named(dashboard, "fan-rpm", i));
		qdev_connect_gpio_out(fanpwm,i,split_fan);
    }

	DeviceState* htr = qdev_new("heater");
	qdev_prop_set_uint8(htr, "thermal_mass_x10",35);
	qdev_prop_set_uint8(htr,"label", 'H');
	sysbus_realize(SYS_BUS_DEVICE(htr), &error_fatal);

    // Heater v - only on if heating. ADC CH11 (Pin B10). Current is In8 (Pin B0)
    DeviceState* vdev = qdev_new("powersource");
    object_property_add_child(periphs, "heater-vmon", OBJECT(vdev));
    qdev_prop_set_uint32(vdev,"mV",23900);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",11));

    vdev = qdev_new("cs30bl");
    object_property_add_child(periphs, "heater-imon", OBJECT(vdev));
    qdev_prop_set_uint32(vdev,"mA", 10);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    qdev_connect_gpio_out_named(vdev, "a_sense",0,qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",8));

	DeviceState* heatpwm = qdev_new("software-pwm");
	sysbus_realize_and_unref(SYS_BUS_DEVICE(heatpwm),&error_fatal);
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM7), "timer", 0, qdev_get_gpio_in_named(heatpwm, "tick-in", 0));
	qemu_irq split_heat = qemu_irq_split(qdev_get_gpio_in_named(htr, "raw-pwm-in",0), qdev_get_gpio_in_named(dashboard, "therm-pwm",0), qdev_get_gpio_in_named(vdev, "pwm-in",0));
	qdev_connect_gpio_out(heatpwm, 0, split_heat);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 6,
		qdev_get_gpio_in_named(heatpwm, "gpio-in",0)
	);

	for (int i=0; i<ARRAY_SIZE(cfg->therm_channels); i++)
    {
        dev = qdev_new("thermistor");
        qdev_prop_set_uint16(dev, "temp",18+i);
        qdev_prop_set_uint16(dev, "table_no", cfg->therm_tables[i]);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
		if (i==0)
		{
			qdev_connect_gpio_out_named(htr, "temp_out",0, qdev_get_gpio_in_named(dev, "thermistor_set_temperature", 0) );
		}
       	//qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", cfg->therm_channels[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in", cfg->therm_channels[i]));
        qdev_connect_gpio_out_named(dev, "temp_out_256x",0, qdev_get_gpio_in_named(dashboard,"therm-temp",i));

    }
    // Check for high-level non configuration arguments like help outputs and handle them.
    if (!arghelper_parseargs())
    {
        // We processed an arg that wants us to quit after it's done.
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }

    DeviceState *lc = qdev_new("loadcell");
    sysbus_realize(SYS_BUS_DEVICE(lc), &error_fatal);

	dev = qdev_new("hx717");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),3)); // EXTR_DATA
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),0,qdev_get_gpio_in(dev, 0)); // EXTR_SCK
	qdev_connect_gpio_out(lc,0, qdev_get_gpio_in_named(dev,"input_x1000",0));

	DeviceState* mux = qdev_new("cbtl3257");
    object_property_add_child(periphs, "mux", OBJECT(mux));
	sysbus_realize_and_unref(SYS_BUS_DEVICE(mux), &error_fatal);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 4,qdev_get_gpio_in_named(mux,"select",0));
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 0,qdev_get_gpio_in_named(mux,"B1",0));
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 1,qdev_get_gpio_in_named(mux,"B1",1));

	if (kernel_len==0 || arghelper_is_arg("no-bridge"))
	{
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 0, qdev_get_gpio_in_named(motor,"dir",0));
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 1, qdev_get_gpio_in_named(motor,"step",0));
	}
	else
	{
		dev = qdev_new("xl-bridge");
		qdev_prop_set_uint8(dev, "device", XL_DEV_T0 + mc->tool_index);
		sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 12, qdev_get_gpio_in_named(dev,"tx-assert",0));
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_USART1),"byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
		qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_USART1),"byte-in", 0));
		//qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_nAC_FAULT, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 12));

		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_DIR,qdev_get_gpio_in_named(mux,"B2",0));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_STEP,qdev_get_gpio_in_named(mux,"B2",1));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_X_UM, qdev_get_gpio_in(lc,0));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_Y_UM, qdev_get_gpio_in(lc,1));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_Z_UM, qdev_get_gpio_in(lc,2));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_CAL_PIN, qdev_get_gpio_in_named(lc, "cal-pin-state", 0));
        qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_P0_PARKED, qdev_get_gpio_in_named(park_sense,"ext-in",0));
        qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_P1_PICKED, qdev_get_gpio_in_named(pick_sense,"ext-in",0));

		qdev_connect_gpio_out_named(dev, "gpio-out", 0, qdev_get_gpio_in_named(motor,"dir",0));
		qdev_connect_gpio_out_named(dev, "gpio-out", 1, qdev_get_gpio_in_named(motor,"step",0));
	}
};

static void xl_extruder_class_init(ObjectClass *oc, void *data)
{
		const xlExtruderData* d = (xlExtruderData*)data;
	    MachineClass *mc = MACHINE_CLASS(oc);
	    mc->desc = d->descr;
	    mc->family = TYPE_XLEXTRUDER_MACHINE,
	    mc->init = prusa_xl_extruder_init;
	    mc->default_ram_size = 0; // 0 = use default RAM from chip.
	    mc->no_parallel = 1;
		mc->no_serial = 1;

		xlExtruderMachineClass* xec = XLEXTRUDER_MACHINE_CLASS(oc);
        xec->hw_type = d->hw_type;
        xec->flash_filename = d->flash_filename;;
        xec->tool_name = d->tool_name;
        xec->tool_index = d->tool_index;
}

#define ADD_TYPEINFO(hwtype, index, vercode) \
    { \
        .name = MACHINE_TYPE_NAME("prusa-xl-extruder-" #vercode "-" #index), \
        .parent = TYPE_XLEXTRUDER_MACHINE, \
        .class_init = xl_extruder_class_init, \
        .class_data = (void*)&xl_extruder_data_##hwtype##index, \
    },

#define ADD_VER_TYPEINFO(hwtype, vercode) \
    ADD_TYPEINFO(hwtype, 4, vercode) \
    ADD_TYPEINFO(hwtype, 3, vercode) \
    ADD_TYPEINFO(hwtype, 2, vercode) \
    ADD_TYPEINFO(hwtype, 1, vercode) \
    ADD_TYPEINFO(hwtype, 0, vercode) 

static const TypeInfo xl_extruder_machine_types[] = {
    {
        .name = TYPE_XLEXTRUDER_MACHINE,
        .parent = TYPE_MACHINE,
        .class_size = sizeof(xlExtruderMachineClass),
        .abstract = true,
    },
    ADD_VER_TYPEINFO(E_STM32G0, 060)
    ADD_VER_TYPEINFO(E_STM32G0_0_4_0, 040)
};

DEFINE_TYPES(xl_extruder_machine_types)
