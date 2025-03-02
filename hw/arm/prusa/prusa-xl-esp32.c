/*
 * Prusa XL embedded ESP32 machine model.
 *
 * Copyright 2024 VintagePC <github.com/vintagepc>
 * 
 * It's just a derived version that adds some convenience wrapping
 * for the control lines and the automatic UART connection from xl-bridge
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
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "sysemu/block-backend.h"
#include "sysemu/sysemu.h"
#include "chardev/char.h"
#include "hw/qdev-properties.h"
#include "hw/core/split-irq.h"
#include "qemu/error-report.h"
#include "parts/xl_bridge.h"

#define FLASH_FN "Prusa_XL_ESP32_flash.bin"
#define FLASH_ID "prusa-xl-esp32-flash"

// Yes, I'm being really lazy about this right now. 
// Long term, this shuold be consolidated with the version in stm32_common.

static bool create_if_not_exist(const char* default_name, uint32_t file_size)
{
	bool exists = true;
	if (access(default_name, R_OK | W_OK) == -1)
	{
#ifndef CONFIG_GCOV
		printf("%s not found - creating it.\n",default_name);
#endif
		// Create it.
		int fd = creat(default_name, S_IRUSR | S_IWUSR);
		exists = (ftruncate(fd, file_size) != -1);
		close(fd);
	}
	return exists;
}

static BlockBackend* get_or_create_drive(BlockInterfaceType interface, int index, const char* default_name, const char* label, uint32_t file_size, Error** errp)
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
#ifndef CONFIG_GCOV
			printf("No -%s drive specified, using default %s\n",
				interface==IF_MTD? "mtdblock" : "pflash",
				default_name);
#endif
			QemuOpts* drive_opts = drive_add(interface, index, default_name, "format=raw");
			dinfo = drive_new(drive_opts, interface, errp);
		}
	}
	return blk_by_legacy_dinfo(dinfo);
}


static void prusa_esp32_init(MachineState *machine)
{
    // Set strap mode so it's not needed on command line.
    GlobalProperty *g;
    g = g_malloc0(sizeof(*g));
    g->driver   = "esp32.gpio";
    g->property = "strap_mode";
    g->value    = "0x0f";
    qdev_prop_register_global(g);

    //  Create chardev handler. We can't pipe bytes directly into the UART like the STM32 side, so it's a bit more complex.
    if (!serial_hd(0))
    {
        QemuOpts *opts;
        opts = qemu_opts_create(qemu_find_opts("chardev"), "prusaxl-esp32", 1, NULL);
                qemu_opt_set(opts, "backend","socket", &error_fatal);
                qemu_opt_set(opts, "path", "/tmp/PXL_ESP32", &error_fatal);
            qemu_chr_new_from_opts(opts, NULL, &error_fatal);
            qemu_opts_del(opts);
    }

    // Create flash storage:
    get_or_create_drive(IF_MTD, 0, FLASH_FN, FLASH_ID,  4U*MiB, &error_fatal);

    DeviceState *dev = qdev_new("xl-bridge");
    qdev_prop_set_uint8(dev, "device", XL_DEV_ESP32);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    // Invoke parent
    ObjectClass *oc = object_class_by_name(MACHINE_TYPE_NAME("esp32"));
    if (oc != NULL) 
    {
        MachineClass *mc = MACHINE_CLASS(oc);
        mc->init(machine);
    }


    // qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
    // qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_UART1),"byte-in", 0));
};

static void xlbuddy_esp32_class_init(ObjectClass *oc, void *data)
{
	    MachineClass *mc = MACHINE_CLASS(oc);
	    mc->desc = "Prusa XL embedded ESP32";
	    mc->family = "xlbuddy-machine",
        mc->no_parallel = true;
        mc->no_serial = true;
        mc->init = prusa_esp32_init;
}

static const TypeInfo xlbuddy_esp32_machine_types[] = {
    {
        .name           = MACHINE_TYPE_NAME("prusa-xl-esp32"),
        .parent         = MACHINE_TYPE_NAME("esp32"),
		.class_init     = xlbuddy_esp32_class_init,
    }
};

DEFINE_TYPES(xlbuddy_esp32_machine_types)
