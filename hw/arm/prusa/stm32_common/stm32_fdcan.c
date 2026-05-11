/*
 * STM32FH503 FDCAN
 *
 * Copyright (c) 2026 VintagePC <github.com/vinagepc>
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "../utility/macros.h"
#include "stm32_types.h"
#include "stm32_common.h"
#include "stm32_fdcan_index.h"
#include "stm32_fdcan_regdata.h"
#include "net/can_emu.h"
#include "trace.h"
#include "hw/core/qdev-properties.h"


OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Fdcan), COM_CLASS_NAME(Fdcan), STM32COM_CAN);


typedef struct COM_STRUCT_NAME(Fdcan) {
    /* <private> */
    STM32Peripheral parent;

	/* <public> */
	/* Memory region */
	MemoryRegion mmio;

	REGDEF_NAME(stm32com,fdcan) regs;

	MemoryRegion *can_sram;

	qemu_irq it[2];

	const stm32_reginfo_t* reginfo;

    CanBusState *bus;
	CanBusClientState bus_client;

} COM_STRUCT_NAME(Fdcan);

typedef struct COM_CLASS_NAME(Fdcan) {
	/* <private> */
	STM32PeripheralClass parent_class;
	const stm32_reginfo_t* var_reginfo;
} COM_CLASS_NAME(Fdcan);


#include "../stm32_registers/generated/stm32c092/FDCAN_reginfo.h"

static const stm32_periph_variant_t stm32_fdcan_variants[] = {
	{TYPE_STM32C092_FDCAN, stm32_c092_fdcan_reginfo},
};

/* Routine which updates the FDCAN's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32com_fdcan_update_irq(COM_STRUCT_NAME(Fdcan) *s)
{

}

// bool ctucan_can_receive(CanBusClientState *client)
// {
//     CtuCanCoreState *s = container_of(client, CtuCanCoreState, bus_client);

//     if (!s->mode_settings.s.ena) {
//         return false;
//     }

//     return true; /* always return true, when operation mode */
// }

// ssize_t ctucan_receive(CanBusClientState *client, const qemu_can_frame *frames,
//                         size_t frames_cnt)
// {
//     CtuCanCoreState *s = container_of(client, CtuCanCoreState, bus_client);
//     static uint8_t rcv[CTUCAN_MSG_MAX_LEN];
//     int i;
//     int ret = -1;
//     const qemu_can_frame *frame = frames;
//     union ctu_can_fd_int_stat int_stat;
//     int_stat.u32 = 0;

//     if (frames_cnt <= 0) {
//         return 0;
//     }

//     ret = ctucan_frame2buff(frame, rcv);

//     if (s->rx_cnt + ret > CTUCAN_RCV_BUF_LEN) { /* Data overrun. */
//         s->status.s.dor = 1;
//         int_stat.s.doi = 1;
//         s->int_stat.u32 |= int_stat.u32 & ~s->int_mask.u32;
//         ctucan_update_irq(s);
//         DPRINTF("Receive FIFO overrun\n");
//         return ret;
//     }
//     s->status.s.idle = 0;
//     s->status.s.rxs = 1;
//     int_stat.s.rxi = 1;
//     if (((s->rx_cnt + 3) & ~3) == CTUCAN_RCV_BUF_LEN) {
//         int_stat.s.rxfi = 1;
//     }
//     s->int_stat.u32 |= int_stat.u32 & ~s->int_mask.u32;
//     s->rx_fr_ctr.s.rx_fr_ctr_val++;
//     s->rx_status_rx_settings.s.rxfrc++;
//     for (i = 0; i < ret; i++) {
//         s->rx_buff[(s->rx_tail_pos + s->rx_cnt) % CTUCAN_RCV_BUF_LEN] = rcv[i];
//         s->rx_cnt++;
//     }
//     s->status.s.rxne = 1;

//     ctucan_update_irq(s);

//     return 1;
// }

// static CanBusClientInfo ctucan_bus_client_info = {
//     .can_receive = ctucan_can_receive,
//     .receive = ctucan_receive,
// };


// int ctucan_connect_to_bus(CtuCanCoreState *s, CanBusState *bus)
// {
//     s->bus_client.info = &ctucan_bus_client_info;

//     if (!bus) {
//         return -EINVAL;
//     }

//     if (can_bus_insert_client(bus, &s->bus_client) < 0) {
//         return -1;
//     }

//     return 0;
// }

// void ctucan_disconnect(CtuCanCoreState *s)
// {
//     can_bus_remove_client(&s->bus_client);
// }


static uint64_t
stm32com_fdcan_read(void *dev, hwaddr offset, unsigned size)
{
    COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	int index = offset >> 2U;
	int shift = offset & 0x3U;

	CHECK_BOUNDS_R_V2(index, RI_END, s->reginfo);

	switch(index) {
		break;
	}

	uint32_t r = s->regs.raw[index];

	ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, shift, 0b110);

	printf("STM32 FDCAN: Unhandled read of 0x%"PRIx64"+%u size %u val 0x%08x\n",offset, shift, size, r);
    return r;
}


static void
stm32com_fdcan_write(void *dev, hwaddr offset, uint64_t data, unsigned size)
{

    COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	int index = offset >> 2U;
	offset &= 0x3U;

	CHECK_BOUNDS_W_V2(index, data , RI_END);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[index], data, size, offset, 0b110);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[index], s->reginfo, index);

	s->regs.raw[index] = data;

	switch (index) {
            break;
	}
    stm32com_fdcan_update_irq(s);
}

static const MemoryRegionOps stm32com_fdcan_ops = {
    .read = stm32com_fdcan_read,
    .write = stm32com_fdcan_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32com_fdcan_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (s->reginfo[i].not_reserved)
			stm32com_fdcan_write(dev, i<<2U, s->reginfo[i].reset_val, 4U);
	}
}

/* DEVICE INITIALIZATION */
static void stm32com_fdcan_realize(DeviceState *dev, Error **errp)
{

}

static const Property stm32_fdcan_properties[] = {
    DEFINE_PROP_LINK("sram",  COM_STRUCT_NAME(Fdcan), can_sram, TYPE_MEMORY_REGION, MemoryRegion*),
};

static void stm32com_fdcan_init(Object *obj)
{

    COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(obj);
    STM32_MR_IO_INIT(&s->mmio, obj, &stm32com_fdcan_ops, s, 1U * KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

	COM_CLASS_NAME(Fdcan) *k = STM32COM_CAN_GET_CLASS(obj);
	s->reginfo = k->var_reginfo;

	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->it[0]);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->it[1]);

    // s->bus = fdcan_init_bus(DEVICE(obj),"fdcan");

}

static const VMStateDescription vmstate_stm32com_fdcan = {
    .name = TYPE_STM32COM_CAN,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Fdcan), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32com_fdcan_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32com_fdcan_reset);
    dc->realize = stm32com_fdcan_realize;
    dc->vmsd = &vmstate_stm32com_fdcan;

	device_class_set_props(dc, stm32_fdcan_properties);

	COM_CLASS_NAME(Fdcan) *k = STM32COM_CAN_CLASS(klass);
	k->var_reginfo = (const stm32_reginfo_t*)data;
}

static TypeInfo stm32com_fdcan_info = {
    .name  = TYPE_STM32COM_CAN,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size  = sizeof(COM_STRUCT_NAME(Fdcan)),
	.class_size = sizeof(COM_CLASS_NAME(Fdcan)),
    // .class_init = stm32com_fdcan_class_init,
    // .instance_init = stm32com_fdcan_init,
	.abstract = true
};

static void stm32com_fdcan_register_types(void)
{
    type_register_static(&stm32com_fdcan_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_fdcan_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_fdcan_variants[i].variant_name,
            .parent     = TYPE_STM32COM_CAN,
			.instance_init = stm32com_fdcan_init,
    		.class_init    = stm32com_fdcan_class_init,
            .class_data = (void *)stm32_fdcan_variants[i].variant_regs,
		};
		type_register_static(&ti);
	}
}

type_init(stm32com_fdcan_register_types)
