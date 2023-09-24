/*
    mmu_bridge.h - (unix) socket transport for MK404 MMU

	Copyright 2023 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "../utility/macros.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "mmu_bridge.h"
#include "qemu/atomic.h"
#include "qemu/option.h"
#include "qemu/config-file.h"

// Messages are composed of some GPIO status bits
// and a serial byte.

#define TYPE_MMUBRIDGE "mmu-bridge"

OBJECT_DECLARE_SIMPLE_TYPE(MMUBridgeState, MMUBRIDGE)



struct MMUBridgeState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/

	// Output backend
	CharBackend chr;
	qemu_irq fs_out;

};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(MMUBridgeState, mmu_bridge, MMUBRIDGE,SYS_BUS_DEVICE,{NULL});

static int mmu_bridge_can_receive(void *opaque)
{
    return 1;
}


static void mmu_bridge_receive(void *opaque, const uint8_t *buf, int size)
{
    MMUBridgeState *s = MMUBRIDGE(opaque);

    switch (buf[0])
    {
        case FS_AUTO_SET:
            qemu_set_irq(s->fs_out, 1);
            printf("MMU FS set\n");
            break;
        case FS_AUTO_CLEAR:
            qemu_set_irq(s->fs_out, 0);
            printf("MMU FS clear\n");
            break;
        default:
            printf("Unknown MMU control char: %c\n", buf[0]);
    }
}

static void mmu_bridge_reset(DeviceState *dev)
{
	MMUBridgeState *s = MMUBRIDGE(dev);
    unsigned char c = RESET;
	qemu_chr_fe_write(&s->chr, &c , 1);
}

static void mmu_bridge_reset_in(void *opaque, int n, int level)
{
    if (level)
    {
        printf("Sent MMU reset\n");
        mmu_bridge_reset(DEVICE(opaque));
    }
}

static void mmu_bridge_finalize(Object *obj)
{
}

#define CHARDEV_NAME "mmu-control"

static void mmu_bridge_realize(DeviceState *dev, Error **errp)
{
    MMUBridgeState *s = MMUBRIDGE(dev);
    Chardev* d=qemu_chr_find(CHARDEV_NAME);
    if (d)
    {
        printf("Found ID MMU control chardev - assigned!");
    }
    else
    {
        QemuOpts *opts;
        printf("Socket ID %s - not found, creating it instead.\n", CHARDEV_NAME);
        opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup(CHARDEV_NAME), 1, NULL);
            qemu_opt_set(opts, "backend","serial", errp);
            qemu_opt_set(opts, "path", g_strdup_printf("/tmp/MK404-MMU-sideband"), errp);
            d = qemu_chr_new_from_opts(opts, NULL, errp);
        qemu_opts_del(opts);
    }
    qemu_chr_fe_init(&s->chr,d, errp);
    qemu_chr_fe_set_handlers(&s->chr, mmu_bridge_can_receive, mmu_bridge_receive, NULL,
        NULL,s,NULL,true);
    qemu_chr_fe_accept_input(&s->chr);
}

static void mmu_bridge_init(Object *obj)
{
    MMUBridgeState *s = MMUBRIDGE(obj);
	DeviceState* dev = DEVICE(obj);
	qdev_init_gpio_in_named(dev, mmu_bridge_reset_in, "reset-in", 1);
	qdev_init_gpio_out_named(dev, &s->fs_out, "fs-out",1);
}

static const VMStateDescription vmstate_mmu_bridge = {
    .name = TYPE_MMUBRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_END_OF_LIST(),
    }
};

static void mmu_bridge_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = mmu_bridge_reset;
    dc->vmsd = &vmstate_mmu_bridge;
	dc->realize = mmu_bridge_realize;
}
