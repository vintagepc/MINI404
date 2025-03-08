/*
 * Prusa xBuddy extension board machine model
 *
 * Copyright 2024 VintagePC <github.com/vintagepc>
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
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/c1_bridge.h"
#include "otp.h"
#include "parts/fan.h"
#include "qapi/qmp/qlist.h"

enum HW_VER
{
	EXT_REV_05,
	EXT_HW_VER_COUNT,
};

#define BOOTLOADER_IMAGE "NOT_SET_YET_FIX_ME.bin"

typedef struct prusa_ext_cfg_t
{

} prusa_ext_cfg_t;

prusa_ext_cfg_t ext_rev05 = {};

static const prusa_ext_cfg_t* ext_cfg_map[EXT_HW_VER_COUNT] =
{
	[EXT_REV_05] = &ext_rev05,
};

static void _prusa_xb_ext_init(MachineState *machine, int index, int type)
{
    DeviceState *dev;

    // Object* periphs = container_get(OBJECT(machine), "/peripheral");

	const prusa_ext_cfg_t* cfg ATTRIBUTE_UNUSED = ext_cfg_map[type];

	OTP_v5 otp_data = { .version = 5, .size = sizeof(OTP_v5),
		.datamatrix = {'4', '5', '5', '8', '-', '5', '0', '0', '0', '0', '0', '1', '9', '0', '0', '5', '2', '5', '9', '9', '9', '9', 0, 0}
	};

	uint32_t* otp_raw = (uint32_t*) &otp_data;

	dev = qdev_new(TYPE_STM32H503xx_SOC);

	// TODO.. can we somehow detect if an extruder is already running and auto-increment the index?
	// maybe with flock on the extuder flash filename?

	hwaddr FLASH_SIZE = stm32_soc_get_flash_size(dev);
	DeviceState* dev_soc = dev;
	qdev_prop_set_string(dev, "flash-file", "pruxa-xbuddy-extension-flash.bin");
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));

	DeviceState* otp = stm32_soc_get_periph(dev, STM32_P_OTP);
    QList *otp_list = qlist_new();
    for (int i = 0; i < 8; i++) {
        qlist_append_int(otp_list, otp_raw[i]);
    }
    qdev_prop_set_array(otp, "otp-data", otp_list);

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


    void* bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, STM32_P_I2C2),
			"i2c");

	// Technically it's a TC6408, but the PCA9557 seems to be identical internally.
	// DeviceState* expander = DEVICE(i2c_slave_create_simple(bus, "pca9557", 0x40));
	i2c_slave_create_simple(bus, "pca9557", 0x40);


	DeviceState* dashboard = qdev_new("2d-dashboard");
    qdev_prop_set_uint8(dashboard, "fans", 3);
    qdev_prop_set_uint8(dashboard, "thermistors", 1);
    qdev_prop_set_string(dashboard, "indicators", "LWSFD");
    qdev_prop_set_string(dashboard, "title", "xBuddy Extension");

	sysbus_realize(SYS_BUS_DEVICE(dashboard), &error_fatal);

	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM2),"pwm_ratio_changed",3,qdev_get_gpio_in_named(dashboard, "led-r",0));
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM2),"pwm_ratio_changed",2,qdev_get_gpio_in_named(dashboard, "led-g",0));
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM2),"pwm_ratio_changed",1,qdev_get_gpio_in_named(dashboard, "led-b",0));
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM2),"pwm_ratio_changed",0,qdev_get_gpio_in_named(dashboard, "led-w",1));
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",0,qdev_get_gpio_in_named(dashboard, "led-w",1));

	// Fan 1 and 2 are T3 CCR2, fan 3 is T3 CCR3
  	uint16_t fan_max_rpms[] = { 5000, 6000, 7000 };
	uint8_t fan_tach_exti_lines[] = { 8, 9, 10};
    uint8_t fan_labels[] = {'1','2', '3'};
	DeviceState* fans[3] = {NULL};
    for (int i=0; i<ARRAY_SIZE(fan_labels); i++)
    {
        fans[i] = dev = qdev_new("fan");
        qdev_prop_set_uint8(dev,"label",fan_labels[i]);
        qdev_prop_set_uint32(dev, "max_rpm",fan_max_rpms[i]);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(dev, "tach-out",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),fan_tach_exti_lines[i]));
		qdev_connect_gpio_out_named(dev, "rpm-out", 0, qdev_get_gpio_in_named(dashboard, "fan-rpm", i));
		if (i == 1)
		{
			qemu_irq split_pwm = qemu_irq_split(
				qdev_get_gpio_in_named(fans[0], "pwm-in", FAN_PWM_INPUT_INVERTED),
				qdev_get_gpio_in_named(fans[1], "pwm-in", FAN_PWM_INPUT_INVERTED)
			);
			qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",2,split_pwm);
			qdev_connect_gpio_out_named(fans[0], "pwm-out", 0, qdev_get_gpio_in_named(dashboard, "fan-pwm",0));
			qdev_connect_gpio_out_named(fans[1], "pwm-out", 0, qdev_get_gpio_in_named(dashboard, "fan-pwm",1));
		}
		else
		{
			qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",3,qdev_get_gpio_in_named(fans[i], "pwm-in", FAN_PWM_INPUT));
			qdev_connect_gpio_out_named(fans[i], "pwm-out", 0, qdev_get_gpio_in_named(dashboard, "fan-pwm",i));
		}
    }

	dev = qdev_new("thermistor");
	qdev_prop_set_uint16(dev, "temp", 40);
	qdev_prop_set_uint16(dev, "table_no", 2009);
	sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
	qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in", 5));
	qdev_connect_gpio_out_named(dev, "temp_out_256x",0, qdev_get_gpio_in_named(dashboard, "therm-temp",0));


	if (kernel_len==0 || arghelper_is_arg("no-bridge"))
	{
	}
	else
	{
		dev = qdev_new("c1-bridge");
		qdev_prop_set_uint8(dev, "device", C1_DEV_EXT);
		sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_USART3), "rts-de", 0, qdev_get_gpio_in_named(dev,"tx-assert",0));
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_USART3),"byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
		qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_USART3),"byte-in", 0));
		//qdev_connect_gpio_out_named(dev, "gpio-out", XLBRIDGE_PIN_nAC_FAULT, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 12));
	}
};

#define ADD_MACHINE(enumentry, str_suffix, shortcode) \
	static void prusa_xb_ext_init_##enumentry(MachineState *machine) \
	{ \
		_prusa_xb_ext_init(machine, -1, enumentry); \
	} \
	static void prusa_xb_ext_machine_init_##enumentry(MachineClass *mc) \
	{ \
		mc->desc = "Prusa xBuddy Extension Board " str_suffix; \
		mc->init = prusa_xb_ext_init_##enumentry; \
		mc->no_serial = 1; \
		mc->no_parallel = 1; \
	} \
	DEFINE_MACHINE("prusa-xbuddy-extension-"#shortcode, prusa_xb_ext_machine_init_##enumentry)

#define ADD_EXT_HWVER(enumentry, vercode) \
ADD_MACHINE(enumentry, "(Rev 05)", vercode); \

ADD_EXT_HWVER(EXT_REV_05, 05);
