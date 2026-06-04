/*
 * Prusa Contactless Offset Sensor board machine model
 *
 * Copyright 2026 VintagePC <github.com/vintagepc>
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
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "qemu/error-report.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/core/loader.h"
#include "hw/misc/led.h"
#include "utility/ArgHelper.h"
#include "system/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/c1_bridge.h"
#include "otp.h"
#include "parts/fan.h"
#include "qobject/qlist.h"

enum HW_VER
{
	CLO_REV_XX,
	CLO_HW_VER_COUNT,
};

#define BOOTLOADER_IMAGE "NOT_SET_YET_FIX_ME.bin"

typedef struct prusa_ext_cfg_t
{

} prusa_clo_cfg_t;

prusa_clo_cfg_t clo_revXX = {};

static const prusa_clo_cfg_t* clo_cfg_map[CLO_HW_VER_COUNT] =
{
	[CLO_REV_XX] = &clo_revXX,
};

static void _prusa_clo_init(MachineState *machine, int index, int type)
{
    DeviceState *dev;

    Object* periphs = machine_get_container("peripheral");

	const prusa_clo_cfg_t* cfg ATTRIBUTE_UNUSED = clo_cfg_map[type];

	//·uint32_t* otp_raw = (uint32_t*) &otp_data;

	dev = qdev_new(TYPE_STM32C092xC_SOC);

	hwaddr FLASH_SIZE = stm32_soc_get_flash_size(dev);
	DeviceState* dev_soc = dev;
	qdev_prop_set_string(dev, "flash-file", "pruxa-clo-flash.bin");
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    qdev_prop_set_uint32(dev,"flash-size", FLASH_SIZE);

    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev_soc));

	// DeviceState* otp = stm32_soc_get_periph(dev, STM32_P_OTP);
    // QList *otp_list = qlist_new();
    // for (int i = 0; i < 8; i++) {
    //     qlist_append_int(otp_list, otp_raw[i]);
    // }
    // qdev_prop_set_array(otp, "otp-data", otp_list);

    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);

    char* kfn = machine->kernel_filename;
    int kernel_len = kfn ? strlen(kfn) : 0;
	if (kernel_len)
    {
		stm32_soc_load_kernel(OBJECT(dev), machine->kernel_filename);
    }

    void* bus = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, STM32_P_I2C1),"i2c");
    dev = qdev_new("ldc1612");
    qdev_prop_set_uint8(dev, "address", 0x2B);
    object_property_add_child(OBJECT(periphs), "ldc1612", OBJECT(dev));
    qdev_realize(dev, bus, &error_fatal);

    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA), 1));

	DeviceState* dashboard = qdev_new("2d-dashboard");
    qdev_prop_set_string(dashboard, "indicators", "U");
    qdev_prop_set_string(dashboard, "title", "CLO Board");

	sysbus_realize(SYS_BUS_DEVICE(dashboard), &error_fatal);

	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),7,qdev_get_gpio_in_named(dashboard, "led-digital",0));

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
	static void prusa_clo_init_##enumentry(MachineState *machine) \
	{ \
		_prusa_clo_init(machine, -1, enumentry); \
	} \
	static void prusa_clo_machine_init_##enumentry(MachineClass *mc) \
	{ \
		mc->desc = "Prusa Contactless Offset Board " str_suffix; \
		mc->init = prusa_clo_init_##enumentry; \
		mc->no_serial = 1; \
		mc->no_parallel = 1; \
	} \
	DEFINE_MACHINE("prusa-clo-"#shortcode, prusa_clo_machine_init_##enumentry)

#define ADD_EXT_HWVER(enumentry, vercode) \
ADD_MACHINE(enumentry, "(Rev xx)", vercode); \

ADD_EXT_HWVER(CLO_REV_XX, xx);
