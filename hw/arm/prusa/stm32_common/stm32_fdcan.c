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
#include "qapi/error.h"
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


typedef struct COM_STRUCT_NAME(Fdcan)
{
	/* <private> */
	STM32Peripheral parent;

	/* <public> */
	MemoryRegion mmio;

	REGDEF_NAME(stm32com,fdcan) regs;

	MemoryRegion *can_sram;

	qemu_irq it[2];

	const stm32_reginfo_t* reginfo;

	CanBusState *bus;
	CanBusClientState bus_client;

} COM_STRUCT_NAME(Fdcan);

typedef struct COM_CLASS_NAME(Fdcan)
{
	/* <private> */
	STM32PeripheralClass parent_class;
	const stm32_reginfo_t* var_reginfo;
} COM_CLASS_NAME(Fdcan);


#include "../stm32_registers/generated/stm32c092/FDCAN_reginfo.h"
#include "../stm32_registers/generated/stm32h503/FDCAN_reginfo.h"

static const stm32_periph_variant_t stm32_fdcan_variants[] =
{
	{TYPE_STM32C092_FDCAN, stm32_c092_fdcan_reginfo},
	{TYPE_STM32H503_FDCAN, stm32_h503_fdcan_reginfo},
};

/* Fixed SRAM layout offsets (byte offsets from SRAMCAN base, per RM0490 Fig 323) */
#define FDCAN_SRAM_FLSSA         0x0000U
#define FDCAN_SRAM_FLESA         0x0070U
#define FDCAN_SRAM_RXF0SA        0x00B0U
#define FDCAN_SRAM_RXF1SA        0x0188U
#define FDCAN_SRAM_EFSA          0x0260U
#define FDCAN_SRAM_TBSA          0x0278U
#define FDCAN_SRAM_RXF0_DEPTH    3U
#define FDCAN_SRAM_RXF1_DEPTH    3U
#define FDCAN_SRAM_TXBUF_CNT     3U
#define FDCAN_SRAM_TXEV_CNT      3U
#define FDCAN_SRAM_RX_ELEM_WORDS 18U
#define FDCAN_SRAM_TX_ELEM_WORDS 18U
#define FDCAN_SRAM_EV_ELEM_WORDS  2U

/* IR bit groups mapped to ILS[6:0] */
static const uint32_t ils_ir_masks[7] =
{
	0x00000007U, /* RXFIFO0: RF0N|RF0F|RF0L (bits 0-2) */
	0x00000038U, /* RXFIFO1: RF1N|RF1F|RF1L (bits 3-5) */
	0x000001C0U, /* SMSG: HPM|TC|TCF (bits 6-8) */
	0x00001E00U, /* TFERR: TFE|TEFN|TEFF|TEFL (bits 9-12) */
	0x0000E000U, /* MISC: TSW|MRAF|TOO (bits 13-15) */
	0x00030000U, /* BERR: ELO|EP (bits 16-17) */
	0x00FC0000U, /* PERR: EW|BO|WDI|PEA|PED|ARA (bits 18-23) */
};

static const uint8_t dlc_to_len[16] =
{
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

/* Routes pending/enabled interrupts through ILS to lines 0/1 and fires IRQs. */
static void stm32com_fdcan_update_irq(COM_STRUCT_NAME(Fdcan) *s)
{
	uint32_t active = s->regs.IR.raw & s->regs.IE.raw;
	uint32_t ils = s->regs.ILS.raw;
	bool line0 = false;
	bool line1 = false;

	for (int g = 0; g < 7; g++)
	{
		if (active & ils_ir_masks[g])
		{
			if ((ils >> g) & 1U)
			{
				line1 = true;
			}
			else
			{
				line0 = true;
			}
		}
	}

	qemu_set_irq(s->it[0], s->regs.ILE.bits.EINT0 && line0);
	qemu_set_irq(s->it[1], s->regs.ILE.bits.EINT1 && line1);
}

/* Writes one received frame into RX FIFO 0. */
static void stm32com_fdcan_rx_frame(COM_STRUCT_NAME(Fdcan) *s, const qemu_can_frame *frame)
{
	if (!s->can_sram)
	{
		return;
	}

	uint32_t *sram_ptr = memory_region_get_ram_ptr(s->can_sram);
	uint32_t f0fl = s->regs.RXF0S.bits.F0FL;

	if (f0fl >= FDCAN_SRAM_RXF0_DEPTH)
	{
		s->regs.RXF0S.bits.RF0L = 1;
		s->regs.IR.bits.RF0L = 1;
		return;
	}

	uint32_t f0pi = s->regs.RXF0S.bits.F0PI;
	uint32_t *elem = sram_ptr + (FDCAN_SRAM_RXF0SA / 4U) + f0pi * FDCAN_SRAM_RX_ELEM_WORDS;

	uint32_t r0 = 0;
	if (frame->can_id & QEMU_CAN_EFF_FLAG)
	{
		r0 = (frame->can_id & 0x1FFFFFFFU) | (1U << 30); /* XTD=1 */
	}
	else
	{
		r0 = ((frame->can_id & 0x7FFU) << 18);
	}
	if (frame->can_id & QEMU_CAN_RTR_FLAG)
	{
		r0 |= (1U << 29);
	}

	/* Reverse-map byte count to DLC code */
	uint8_t len = frame->can_dlc;
	uint8_t dlc = len;
	if (len > 8)
	{
		if (len <= 12)       { dlc = 9; len = 12; }
		else if (len <= 16)  { dlc = 10; len = 16; }
		else if (len <= 20)  { dlc = 11; len = 20; }
		else if (len <= 24)  { dlc = 12; len = 24; }
		else if (len <= 32)  { dlc = 13; len = 32; }
		else if (len <= 48)  { dlc = 14; len = 48; }
		else                 { dlc = 15; len = 64; }
	}
	uint32_t r1 = (uint32_t)dlc << 16;

	elem[0] = r0;
	elem[1] = r1;
	memcpy(&elem[2], frame->data, len);

	s->regs.RXF0S.bits.F0PI = (f0pi + 1) % FDCAN_SRAM_RXF0_DEPTH;
	s->regs.RXF0S.bits.F0FL++;

	if (s->regs.RXF0S.bits.F0FL >= FDCAN_SRAM_RXF0_DEPTH)
	{
		s->regs.RXF0S.bits.F0F = 1;
		s->regs.IR.bits.RF0F = 1;
	}
	s->regs.IR.bits.RF0N = 1;
}

/* Processes pending TX buffers: sends frames and updates status registers. */
static void stm32com_fdcan_process_tx(COM_STRUCT_NAME(Fdcan) *s, uint32_t buf_mask)
{
	if (!s->can_sram)
	{
		return;
	}

	uint32_t *sram_ptr = memory_region_get_ram_ptr(s->can_sram);
	bool lbck = s->regs.TEST.bits.LBCK;

	for (int i = 0; i < (int)FDCAN_SRAM_TXBUF_CNT; i++)
	{
		if (!(buf_mask & (1U << i)))
		{
			continue;
		}

		uint32_t *elem = sram_ptr + (FDCAN_SRAM_TBSA / 4U) + (uint32_t)i * FDCAN_SRAM_TX_ELEM_WORDS;
		uint32_t t0 = elem[0];
		uint32_t t1 = elem[1];

		qemu_can_frame frame;
		memset(&frame, 0, sizeof(frame));

		bool xtd = (t0 >> 30) & 1;
		bool rtr = (t0 >> 29) & 1;
		uint8_t dlc = (t1 >> 16) & 0xFU;
		uint8_t data_len = dlc_to_len[dlc];

		if (xtd)
		{
			frame.can_id = (t0 & 0x1FFFFFFFU) | QEMU_CAN_EFF_FLAG;
		}
		else
		{
			frame.can_id = (t0 >> 18) & 0x7FFU;
		}
		if (rtr)
		{
			frame.can_id |= QEMU_CAN_RTR_FLAG;
		}
		frame.can_dlc = data_len;
		memcpy(frame.data, &elem[2], data_len);

		if (s->bus && !lbck)
		{
			can_bus_client_send(&s->bus_client, &frame, 1);
		}
		if (lbck)
		{
			stm32com_fdcan_rx_frame(s, &frame);
		}

		s->regs.TXBRP.raw &= ~(1U << i);
		s->regs.TXBTO.raw |= (1U << i);
		s->regs.IR.bits.TC = 1;

		/* Write TX event FIFO entry if EFC=1 */
		bool efc = (t1 >> 23) & 1;
		if (efc)
		{
			uint32_t effl = s->regs.TXEFS.bits.EFFL;
			if (effl < FDCAN_SRAM_TXEV_CNT)
			{
				uint32_t efpi = s->regs.TXEFS.bits.EFPI;
				uint32_t *ev = sram_ptr + (FDCAN_SRAM_EFSA / 4U) + efpi * FDCAN_SRAM_EV_ELEM_WORDS;
				ev[0] = t0;
				ev[1] = t1;
				s->regs.TXEFS.bits.EFPI = (efpi + 1) % FDCAN_SRAM_TXEV_CNT;
				s->regs.TXEFS.bits.EFFL++;
				if (s->regs.TXEFS.bits.EFFL >= FDCAN_SRAM_TXEV_CNT)
				{
					s->regs.TXEFS.bits.EFF = 1;
				}
				s->regs.IR.bits.TEFN = 1;
			}
			else
			{
				s->regs.IR.bits.TEFL = 1;
			}
		}

		/* Advance FIFO put index */
		uint32_t tfqpi = s->regs.TXFQS.bits.TFQPI;
		s->regs.TXFQS.bits.TFQPI = (tfqpi + 1) % FDCAN_SRAM_TXBUF_CNT;
		/* Synchronous completion: free level always stays at max */
		s->regs.TXFQS.bits.TFFL = FDCAN_SRAM_TXBUF_CNT;
		s->regs.TXFQS.bits.TFQF = 0;
	}
}

static bool stm32com_fdcan_can_receive(CanBusClientState *client)
{
	COM_STRUCT_NAME(Fdcan) *s = container_of(client, COM_STRUCT_NAME(Fdcan), bus_client);
	return !s->regs.CCCR.bits.INIT;
}

static ssize_t stm32com_fdcan_receive(CanBusClientState *client,
	const qemu_can_frame *frames, size_t frames_cnt)
{
	COM_STRUCT_NAME(Fdcan) *s = container_of(client, COM_STRUCT_NAME(Fdcan), bus_client);

	if (frames_cnt == 0)
	{
		return 0;
	}
	stm32com_fdcan_rx_frame(s, frames);
	stm32com_fdcan_update_irq(s);
	return 1;
}

static CanBusClientInfo stm32com_fdcan_bus_client_info =
{
	.can_receive = stm32com_fdcan_can_receive,
	.receive = stm32com_fdcan_receive,
};


static uint64_t
stm32com_fdcan_read(void *dev, hwaddr offset, unsigned size)
{
	COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	int index = offset >> 2U;
	int shift = offset & 0x3U;

	CHECK_BOUNDS_R_V2(index, RI_END, s->reginfo);

	switch(index)
	{
		break;
	}

	uint32_t r = s->regs.raw[index];

	ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, shift, 0b110);

	return r;
}


static void
stm32com_fdcan_write(void *dev, hwaddr offset, uint64_t data, unsigned size)
{
	COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	int index = offset >> 2U;
	offset &= 0x3U;

	CHECK_BOUNDS_W_V2(index, data, RI_END);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[index], data, size, offset, 0b110);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[index], s->reginfo, index);

	switch (index)
	{
		case RI_IR:
			/* W1C: writing 1 clears the bit */
			s->regs.IR.raw &= ~data;
			break;

		case RI_TXBRP:
		case RI_TXBTO:
		case RI_TXBCF:
		case RI_TXFQS:
		case RI_RXF0S:
		case RI_RXF1S:
		case RI_HPMS:
		case RI_ECR:
		case RI_PSR:
			/* Read-only hardware status registers — ignore writes */
			break;

		case RI_CCCR:
		{
			bool prev_cce = s->regs.CCCR.bits.CCE;
			bool new_init = (data >> 0) & 1;
			bool new_cce  = (data >> 1) & 1;

			/* CCE can only be set/kept when INIT=1 */
			if (!new_init)
			{
				new_cce = false;
			}

			/* Entering config mode (CCE 0→1): reset error counters */
			if (!prev_cce && new_cce)
			{
				s->regs.ECR.raw = s->reginfo[RI_ECR].reset_val;
				s->regs.PSR.raw = s->reginfo[RI_PSR].reset_val;
			}

			data = (data & ~0x3U) | (new_init ? 1U : 0U) | (new_cce ? 2U : 0U);
			s->regs.CCCR.raw = data;
			break;
		}

		case RI_TXBAR:
		{
			uint32_t req = data & 0x7U;
			s->regs.TXBRP.raw |= req;
			stm32com_fdcan_process_tx(s, req);
			break;
		}

		case RI_TXBCR:
		{
			/* Cancel pending requests and mark as cancelled */
			uint32_t cancel = data & s->regs.TXBRP.raw;
			s->regs.TXBRP.raw &= ~cancel;
			s->regs.TXBCF.raw |= cancel;
			if (cancel)
			{
				s->regs.IR.bits.TCF = 1;
			}
			break;
		}

		case RI_RXF0A:
		{
			uint32_t ack = data & 0x7U;
			s->regs.RXF0S.bits.F0GI = (ack + 1U) % FDCAN_SRAM_RXF0_DEPTH;
			if (s->regs.RXF0S.bits.F0FL > 0)
			{
				s->regs.RXF0S.bits.F0FL--;
			}
			if (s->regs.RXF0S.bits.F0FL < FDCAN_SRAM_RXF0_DEPTH)
			{
				s->regs.RXF0S.bits.F0F = 0;
			}
			break;
		}

		case RI_RXF1A:
		{
			uint32_t ack = data & 0x7U;
			s->regs.RXF1S.bits.F1GI = (ack + 1U) % FDCAN_SRAM_RXF1_DEPTH;
			if (s->regs.RXF1S.bits.F1FL > 0)
			{
				s->regs.RXF1S.bits.F1FL--;
			}
			if (s->regs.RXF1S.bits.F1FL < FDCAN_SRAM_RXF1_DEPTH)
			{
				s->regs.RXF1S.bits.F1F = 0;
			}
			break;
		}

		case RI_TXEFA:
		{
			uint32_t ack = data & 0x3U;
			s->regs.TXEFS.bits.EFGI = (ack + 1U) % FDCAN_SRAM_TXEV_CNT;
			if (s->regs.TXEFS.bits.EFFL > 0)
			{
				s->regs.TXEFS.bits.EFFL--;
			}
			if (s->regs.TXEFS.bits.EFFL < FDCAN_SRAM_TXEV_CNT)
			{
				s->regs.TXEFS.bits.EFF = 0;
			}
			break;
		}

		default:
			s->regs.raw[index] = data;
			break;
	}

	stm32com_fdcan_update_irq(s);
}

static const MemoryRegionOps stm32com_fdcan_ops =
{
	.read = stm32com_fdcan_read,
	.write = stm32com_fdcan_write,
	.endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32com_fdcan_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i = 0; i < RI_END; i++)
	{
		if (s->reginfo[i].not_reserved)
		{
			s->regs.raw[i] = s->reginfo[i].reset_val;
		}
	}

	stm32com_fdcan_update_irq(s);
}

/* DEVICE INITIALIZATION */
static void stm32com_fdcan_realize(DeviceState *dev, Error **errp)
{
	COM_STRUCT_NAME(Fdcan) *s = STM32COM_CAN(dev);

	if (s->bus)
	{
		s->bus_client.info = &stm32com_fdcan_bus_client_info;
		if (can_bus_insert_client(s->bus, &s->bus_client) < 0)
		{
			error_setg(errp, "Failed to connect FDCAN to CAN bus");
			return;
		}
	}
}

static const Property stm32_fdcan_properties[] =
{
	DEFINE_PROP_LINK("sram",   COM_STRUCT_NAME(Fdcan), can_sram, TYPE_MEMORY_REGION, MemoryRegion*),
	DEFINE_PROP_LINK("canbus", COM_STRUCT_NAME(Fdcan), bus,      TYPE_CAN_BUS,       CanBusState*),
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
}

static const VMStateDescription vmstate_stm32com_fdcan =
{
	.name = TYPE_STM32COM_CAN,
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = (VMStateField[])
	{
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

static TypeInfo stm32com_fdcan_info =
{
	.name          = TYPE_STM32COM_CAN,
	.parent        = TYPE_STM32_PERIPHERAL,
	.instance_size = sizeof(COM_STRUCT_NAME(Fdcan)),
	.class_size    = sizeof(COM_CLASS_NAME(Fdcan)),
	.abstract      = true
};

static void stm32com_fdcan_register_types(void)
{
	type_register_static(&stm32com_fdcan_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_fdcan_variants); ++i)
	{
		TypeInfo ti =
		{
			.name          = stm32_fdcan_variants[i].variant_name,
			.parent        = TYPE_STM32COM_CAN,
			.instance_init = stm32com_fdcan_init,
			.class_init    = stm32com_fdcan_class_init,
			.class_data    = (void *)stm32_fdcan_variants[i].variant_regs,
		};
		type_register_static(&ti);
	}
}

type_init(stm32com_fdcan_register_types)
