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
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/xl_bridge.h"

#define BOOTLOADER_IMAGE "bl_dwarf.elf.bin"

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

static const char* FLASH_NAMES[] = {
	"Prusa_XL_Dwarf_0_flash.bin",
	"Prusa_XL_Dwarf_1_flash.bin",
	"Prusa_XL_Dwarf_2_flash.bin",
	"Prusa_XL_Dwarf_3_flash.bin",
	"Prusa_XL_Dwarf_4_flash.bin",
	"Prusa_XL_Dwarf_5_flash.bin",
};

static const char* TOOL_NAMES[] = {
	"Dwarf 0",
	"Dwarf 1",
	"Dwarf 2",
	"Dwarf 3",
	"Dwarf 4",
	"Dwarf 5",
};

static void _prusa_xl_extruder_init(MachineState *machine, int index, int type)
{
    DeviceState *dev;

    Object* periphs = container_get(OBJECT(machine), "/peripheral");

	const prusa_xl_e_cfg_t* cfg = extruder_cfg_map[type];

	dev = qdev_new(TYPE_STM32G070xB_SOC);

	// TODO.. can we somehow detect if an extruder is already running and auto-increment the index?
	// maybe with flock on the extuder flash filename?

	hwaddr FLASH_SIZE = stm32_soc_get_flash_size(dev);
	DeviceState* dev_soc = dev;
	qdev_prop_set_string(dev, "flash-file", FLASH_NAMES[index]);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);

    char* kfn = machine->kernel_filename;
    int kernel_len = kfn ? strlen(kfn) : 0;
    if (kernel_len >3 && strncmp(kfn + (kernel_len-3), "bbf",3) == 0 )
    {
        // TODO... use initrd_image as a bootloader alternative?
        struct stat bootloader;
        if (stat(BOOTLOADER_IMAGE,&bootloader))
        {
            error_setg(&error_fatal, "No %s file found. It is required to use a .bbf file!",BOOTLOADER_IMAGE);
            return;
        }
        // BBF has an extra 64b header we need to prune. Rather than modify it or use a temp file, offset it
        // by -64 bytes and rely on the bootloader clobbering it.
        load_image_targphys(machine->kernel_filename,0x08000000,get_image_size(machine->kernel_filename));
        armv7m_load_kernel(ARM_CPU(first_cpu),
            BOOTLOADER_IMAGE, 0,
            FLASH_SIZE);
    }
    else // Raw bin or ELF file, load directly.
    {
        armv7m_load_kernel(ARM_CPU(first_cpu),
                        machine->kernel_filename, 0,
                        FLASH_SIZE);
    }



	DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

	DeviceState* dashboard = qdev_new("2d-dashboard");
    qdev_prop_set_uint8(dashboard, "fans", 2);
    qdev_prop_set_uint8(dashboard, "thermistors", 3);
    qdev_prop_set_string(dashboard, "indicators", "LPF");
    qdev_prop_set_string(dashboard, "title", TOOL_NAMES[index]);

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

	// hotend = fan1
    // print fan = fan0
    uint16_t fan_max_rpms[] = { 5000, 6000 };
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
	qdev_prop_set_uint8(htr, "thermal_mass_x10",30);
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
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",cfg->therm_channels[i]));
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
		qdev_prop_set_uint8(dev, "device", XL_DEV_T0 + index);
		sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 12, qdev_get_gpio_in_named(dev,"tx-assert",0));
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
		qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-in", 0));
		//qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_nAC_FAULT, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 12));

		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_DIR,qdev_get_gpio_in_named(mux,"B2",0));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_E_STEP,qdev_get_gpio_in_named(mux,"B2",1));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_Z_UM, qdev_get_gpio_in(lc,0));

		qdev_connect_gpio_out_named(dev, "gpio-out", 0, qdev_get_gpio_in_named(motor,"dir",0));
		qdev_connect_gpio_out_named(dev, "gpio-out", 1, qdev_get_gpio_in_named(motor,"step",0));
	}
};

#define ADD_MACHINE(enumentry, index, str_suffix, shortcode) \
	static void prusa_xl_extruder_init_##enumentry##index(MachineState *machine) \
	{ \
		_prusa_xl_extruder_init(machine, index, enumentry); \
	} \
	static void prusa_xl_extruder_machine_init_##enumentry##index(MachineClass *mc) \
	{ \
		mc->desc = "Prusa XL Extruder Board " str_suffix; \
		mc->init = prusa_xl_extruder_init_##enumentry##index; \
		mc->no_serial = 1; \
		mc->no_parallel = 1; \
	} \
	DEFINE_MACHINE("prusa-xl-extruder-"#shortcode"-" #index, prusa_xl_extruder_machine_init_##enumentry##index)

#define ADD_E_HWVER(enumentry, vercode) \
ADD_MACHINE(enumentry, 0, "(First Tool)", 	vercode); \
ADD_MACHINE(enumentry, 1, "(Second Tool)",	vercode); \
ADD_MACHINE(enumentry, 2, "(Third Tool)",	vercode); \
ADD_MACHINE(enumentry, 3, "(Fourth Tool)",	vercode); \
ADD_MACHINE(enumentry, 4, "(Fifth Tool)",	vercode);

ADD_E_HWVER(E_STM32G0, 060);
ADD_E_HWVER(E_STM32G0_0_4_0, 040);
