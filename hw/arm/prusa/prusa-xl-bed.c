/*
 * Prusa XL Bed mainboard machine model
 *
 * Copyright 2021-3 VintagePC <github.com/vintagepc>
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
#include "hw/qdev-properties.h"
#include "hw/arm/armv7m.h"
#include "hw/core/split-irq.h"
#include "qemu/error-report.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/xl_bridge.h"

#define BOOTLOADER_IMAGE "bootloader.bin"

// Upper 8 pins are GPIO
typedef uint16_t stm_pin;

static uint8_t BANK(stm_pin pin) {
    return pin >> 8U;
}

static uint8_t PIN(stm_pin pin) {
    return pin & 0xFFU;
}

#define STM_PIN(gpio,num) ((gpio & 0xFF)<< 8) | (num & 0xFF)

enum HW_TYPE {
	B_STM32G0,
	B_STM32G0_v0_6_0,
};

static void prusa_xl_bed_init(MachineState *machine, int hw_type)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_STM32G070xB_SOC);
	hwaddr FLASH_SIZE = stm32_soc_get_flash_size(dev);

	qdev_prop_set_string(dev, "flash-file", "Prusa_XL_ModBed_flash.bin");
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
	qdev_prop_set_uint16(stm32_soc_get_periph(dev, STM32_P_GPIOC), "idr-mask", 0x0D);
	qdev_prop_set_uint16(stm32_soc_get_periph(dev, STM32_P_GPIOC), "idr-force", 0x0F);

    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);

	DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

    int kernel_len = strlen(machine->kernel_filename);
    if (kernel_len >3)
    {
        const char* kernel_ext = machine->kernel_filename+(kernel_len-3);
        if (strncmp(kernel_ext, "bbf",3)==0)
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
            load_image_targphys(machine->kernel_filename,0x20000-64,get_image_size(machine->kernel_filename));
            armv7m_load_kernel(ARM_CPU(first_cpu),
                BOOTLOADER_IMAGE,
                FLASH_SIZE);
        }
        else // Raw bin or ELF file, load directly.
        {
            armv7m_load_kernel(ARM_CPU(first_cpu),
                            machine->kernel_filename,
                            FLASH_SIZE);
        }
    }

	DeviceState* dev_soc = dev;
// HACK
	// SOC->usarts[0].rs485_dest = 0x08;
	// SOC->usarts[0].do_rs485 = true;

	DeviceState* visuals = qdev_new("modular-bed-visuals");
	sysbus_realize(SYS_BUS_DEVICE(visuals), &error_fatal);

	// Note with v0.7.0 HB 2 moved from pin C10 to C8
	static const stm_pin g0_bed_outs[16] = {
		STM_PIN(STM32_P_GPIOB, 4),	STM_PIN(STM32_P_GPIOB, 5), STM_PIN(STM32_P_GPIOC, 8), 	STM_PIN(STM32_P_GPIOB, 9),
		STM_PIN(STM32_P_GPIOD, 1), 	STM_PIN(STM32_P_GPIOD, 2), STM_PIN(STM32_P_GPIOD, 0), 	STM_PIN(STM32_P_GPIOC, 9),
		STM_PIN(STM32_P_GPIOB,13), 	STM_PIN(STM32_P_GPIOA, 8), STM_PIN(STM32_P_GPIOB, 15), 	STM_PIN(STM32_P_GPIOB,14),
		STM_PIN(STM32_P_GPIOC,13), 	STM_PIN(STM32_P_GPIOA,11), STM_PIN(STM32_P_GPIOC,15), 	STM_PIN(STM32_P_GPIOC,14)
	};

	static const stm_pin g0_bed_outs_v060[16] = {
		STM_PIN(STM32_P_GPIOB, 4),	STM_PIN(STM32_P_GPIOB, 5), STM_PIN(STM32_P_GPIOC, 10), 	STM_PIN(STM32_P_GPIOB, 9),
		STM_PIN(STM32_P_GPIOD, 1), 	STM_PIN(STM32_P_GPIOD, 2), STM_PIN(STM32_P_GPIOD, 0), 	STM_PIN(STM32_P_GPIOC, 9),
		STM_PIN(STM32_P_GPIOB,13), 	STM_PIN(STM32_P_GPIOA, 8), STM_PIN(STM32_P_GPIOB, 15), 	STM_PIN(STM32_P_GPIOB,14),
		STM_PIN(STM32_P_GPIOC,13), 	STM_PIN(STM32_P_GPIOA,11), STM_PIN(STM32_P_GPIOC,15), 	STM_PIN(STM32_P_GPIOC,14)
	};

	// Also note: v0.6.0 has an inverted overcurrent reset line. Not that it matters right now, because we don't have
	// an OC latch defined here.

	//static uint8_t g0_adcs[16] = {15, 14, 4, 3, 2, 0, 8, 5, 7, 8, 9, 10, 11, 12, 13, 13};
	// TODO - mux bed 14/15
	static const uint8_t g0_adcs[16] = {7, 17, 18, 8, 9, 9, 11, 11, 16, 15, 4, 3, 2, 0, 1, 5};

	//Note: Ch6 is current A, Ch 10 is currentB.
	// 0-7 are on A, 8-15 on B.
	static const uint8_t current_adcs[2] = {6, 10};
	DeviceState* current_sense[2];
	for (int i=0; i<2; i++)
	{
		current_sense[i] = qdev_new("current-sum");
	}

	const stm_pin* bed_outs = NULL;
	switch (hw_type)
	{
		case B_STM32G0:
			bed_outs = g0_bed_outs;
			break;
		case B_STM32G0_v0_6_0:
			bed_outs = g0_bed_outs_v060;
			break;
	}
	DeviceState* mux = qdev_new("hc4052");
	qdev_prop_set_uint8(mux, "start_channel",1);

    sysbus_realize(SYS_BUS_DEVICE(mux), &error_fatal);
	// 14 = 01, 15 = 10 for selection. There is no actual 4052 mux here, I'm just reusing the part because it works.
	// Update - now it's just 1 pin to switch between 12/13 (1Y) and 14/15 (2Y)
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),6,qdev_get_gpio_in_named(mux,"select",0)); // S0
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", 9,  qdev_get_gpio_in_named(mux, "adc_read_request",0));
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", 11,  qdev_get_gpio_in_named(mux, "adc_read_request",1));
	qdev_connect_gpio_out(mux,0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",9));
	qdev_connect_gpio_out(mux,1, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",11));

	static uint8_t index2vis[16] = {5, 4, 0, 1, 6, 2, 3, 7, 10, 15, 11, 14, 9, 13, 12, 8};
	//static uint8_t index2vis[16] = {0,1,2 ,3 ,4,5 ,6,7,8,9,10,11,12,13,14,15};
	//static uint8_t index2vis[16] = {6,3,10,15,9,12,5,0,2,7,11,14,13,8 ,4 ,1 };

	DeviceState* pwmtest = qdev_new("software-pwm");
	sysbus_realize_and_unref(SYS_BUS_DEVICE(pwmtest),&error_fatal);
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3), "timer", 0, qdev_get_gpio_in_named(pwmtest, "tick-in", 0));

	uint16_t table = 65535;
	for (int i=0; i<16; i++)
    {
        dev = qdev_new("thermistor");
        qdev_prop_set_uint16(dev, "temp", 20);
        qdev_prop_set_uint16(dev, "table_no", table);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
		if (i==4 || i == 5)
		{
			// Map so that 4 is on 1Y1, 5 is 1Y2.
			qdev_connect_gpio_out_named(mux,"1Y_read", i-4,  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
			qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(mux,"1Y", i - 4));
		}
		else if (i==6 || i == 7)
		{
			// Map so that 6 is on 2Y1, 7 is 2Y2.
			qdev_connect_gpio_out_named(mux,"2Y_read", i-6,  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
			qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(mux,"2Y", i - 6));
		}
		else
		{
			qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", g0_adcs[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
			qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",g0_adcs[i]));
		}

		qdev_connect_gpio_out_named(dev, "temp_out_256x",0, qdev_get_gpio_in_named(visuals, "temp-in",index2vis[i]));

		DeviceState* dev2 = qdev_new("heater");
		qdev_prop_set_uint8(dev2, "thermal_mass_x10",10);
		qdev_prop_set_uint8(dev2,"label", 'A'+i);
		qdev_prop_set_uint16(dev2,"resistance_x100", 1400);
		sysbus_realize(SYS_BUS_DEVICE(dev2), &error_fatal);
		gchar* name = g_strdup_printf("heater[%d]", i>5 ? i-6 : i);
		object_property_set_link(OBJECT(current_sense[i>5 ? 0 : 1]), name, OBJECT(dev2), &error_fatal);
		g_free(name);
		qemu_irq split_pwm = qemu_irq_split( qdev_get_gpio_in_named(dev2, "raw-pwm-in", 0), qdev_get_gpio_in_named(visuals, "heat-in",index2vis[i]));
		qdev_connect_gpio_out(pwmtest, i, split_pwm);
		qdev_connect_gpio_out(
			stm32_soc_get_periph(dev_soc, BANK(bed_outs[i])),
			PIN(bed_outs[i]),
			qdev_get_gpio_in_named(pwmtest, "gpio-in", i));
		qdev_connect_gpio_out_named(dev2, "temp_out",0, qdev_get_gpio_in_named(dev, "thermistor_set_temperature",0));
    }

	for (int i=0; i<2; i++)
	{
		sysbus_realize_and_unref(SYS_BUS_DEVICE(current_sense[i]), &error_fatal);
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", current_adcs[i],  qdev_get_gpio_in_named(current_sense[i], "adc_read_request",0));
    	qdev_connect_gpio_out_named(current_sense[i], "adc_out", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",current_adcs[i]));
	}

	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF), 6, qdev_get_gpio_in_named(visuals, "led-in",0));

	dev = qdev_new("powersource");
	sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
	qdev_connect_gpio_out_named(dev, "panic",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),11));

    // Needs to come last because it has the scripting engine setup.
    dev = qdev_new("p404-scriptcon");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    // Check for high-level non configuration arguments like help outputs and handle them.
    if (!arghelper_parseargs())
    {
        // We processed an arg that wants us to quit after it's done.
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }

	if (!arghelper_is_arg("no-bridge"))
	{
		dev = qdev_new("xl-bridge");
		qdev_prop_set_uint8(dev, "device", XL_DEV_BED);
		sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 6, qdev_get_gpio_in_named(dev,"tx-assert",0));
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
		qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-in", 0));
		qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_nAC_FAULT, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC), 11));
	}


};

static void prusa_xl_bed_g0_init(MachineState *machine)
{
	prusa_xl_bed_init(machine, B_STM32G0);
}

static void prusa_xl_bed_g0_v060_init(MachineState *machine)
{
	prusa_xl_bed_init(machine, B_STM32G0_v0_6_0);
}

static void prusa_xl_bed_g0_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa XL Modular Bed Board v0.7.0";
    mc->init = prusa_xl_bed_g0_init;
	mc->no_serial = 1;
	mc->no_parallel = 1;
}

static void prusa_xl_bed_g0_v060_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa XL Modular Bed Board v0.6.0";
    mc->init = prusa_xl_bed_g0_v060_init;
	mc->no_serial = 1;
	mc->no_parallel = 1;
}

DEFINE_MACHINE("prusa-xl-bed-070", prusa_xl_bed_g0_machine_init)
DEFINE_MACHINE("prusa-xl-bed-060", prusa_xl_bed_g0_v060_machine_init)
