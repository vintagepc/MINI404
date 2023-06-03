/*
    stm32_dma.c - DMA for STM32x0.
	So far the DMA layout is common to the following chips:
	- F030 series,
	- G070 series,

	Copyright 2021-3 VintagePC <https://github.com/vintagepc/>

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
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "stm32_rcc_if.h"


// 5 for F030, 7 for G070
#define STM32_COM_DMA_MAX_CHAN 7
#define STM32_COM_DMA_CHAN_REGS 5

#include "stm32_dma_regdata.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Dma), COM_CLASS_NAME(Dma), STM32COM_DMA);

#define _STM32_DMA_INT_BITSET(chan) \
	REG_B32(_JOIN2R(GIF,chan)); \
	REG_B32(_JOIN2R(TCIF,chan)); \
	REG_B32(_JOIN2R(HTIF,chan)); \
	REG_B32(_JOIN2R(TEIF,chan));


REGDEF_BLOCK_BEGIN()
	_STM32_DMA_INT_BITSET(1)
	_STM32_DMA_INT_BITSET(2)
	_STM32_DMA_INT_BITSET(3)
	_STM32_DMA_INT_BITSET(4)
	_STM32_DMA_INT_BITSET(5)
	_STM32_DMA_INT_BITSET(6)
	_STM32_DMA_INT_BITSET(7)
REGDEF_BLOCK_END(dma, interrupt)

#undef _STM32_DMA_INT_BITSET

REGDEF_BLOCK_BEGIN()
	REG_B32(EN);
	REG_B32(TCIE);
	REG_B32(HTIE);
	REG_B32(TEIE);
	REG_B32(DIR);
	REG_B32(CIRC);
	REG_B32(PINC);
	REG_B32(MINC);
	REG_K32(PSIZE,2);
	REG_K32(MSIZE,2);
	REG_K32(PL, 2);
	REG_B32(MEM2MEM);
	REG_R(17);
REGDEF_BLOCK_END(dma, ccr)

#define _STM32_DMA_CHAN_BLK(x) \
	REGDEF_NAME(dma, ccr) _JOIN2R(CCR,x); \
	REG_S32(NDTR, 16) _JOIN2R(CNDTR,x); \
	uint32_t _JOIN2R(CPAR,x); \
	uint32_t _JOIN2R(CMAR,x); \
	uint32_t _JOIN2R(_unusedblk,x);


typedef struct COM_STRUCT_NAME(Dma) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

	MemoryRegion* cpu_mr;
	AddressSpace cpu_as;

    union {
        struct {
			REGDEF_NAME(dma, interrupt) ISR;		//0x00
			REGDEF_NAME(dma, interrupt) IFCR;		//0x04
			_STM32_DMA_CHAN_BLK(1)	 				//0x08 - 0x18
			_STM32_DMA_CHAN_BLK(2)	 				//0x1C - 0x2C
			_STM32_DMA_CHAN_BLK(3)	 				//0x30 - 0x40
			_STM32_DMA_CHAN_BLK(4)	 				//0x44 - 0x54
			_STM32_DMA_CHAN_BLK(5)	 				//0x58 - 0x68
			_STM32_DMA_CHAN_BLK(6)	 				//0x6C - 0x7C
			_STM32_DMA_CHAN_BLK(7)	 				//0x80 - 0x8C
		} defs;
        uint32_t raw[RI_END];
    } regs;

	uint32_t original_ndtrs[STM32_COM_DMA_MAX_CHAN];
	uint32_t original_cmars[STM32_COM_DMA_MAX_CHAN];
	uint32_t original_cpars[STM32_COM_DMA_MAX_CHAN];

	QEMUTimer* dma_timer;
	uint8_t dma_pending[STM32_COM_DMA_MAX_CHAN];

	qemu_irq irq[STM32_COM_DMA_MAX_CHAN];

	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Dma);

#undef _STM32_DMA_CHAN_BLK

enum dma_dir
{
	DIR_P2M,
	DIR_M2P,
};

enum interrupt_bits
{
	INT_GI = 1,
	INT_TC = 2,
	INT_HT = 4,
	INT_TE = 8,
	INT_ALL = 14,
	INT_BLKSIZE = 4
};

static const uint8_t dma_xfer_size_b[4] = {1, 2, 4, 0};

typedef struct COM_CLASS_NAME(Dma) {
	SysBusDeviceClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Dma);

#define CHAN_REGINFO(x)\
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CCR] = {.mask = 0x7FFF }, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CNDTR] = {.mask = UINT16_MAX }, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CPAR] = {.mask = UINT32_MAX}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CMAR] = {.mask = UINT32_MAX}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_END] = {.is_reserved = true}

#define CHAN_RESERVED(x)\
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CCR] = {.is_reserved = true}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CNDTR] = {.is_reserved = true}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CPAR] = {.is_reserved = true}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_CMAR] = {.is_reserved = true}, \
	[_JOIN2R(RI_CHAN,x)+CH_OFF_END] = {.is_reserved = true}

static const stm32_reginfo_t stm32f030_dma_reginfo[RI_END] =
{
	[RI_ISR] = {.mask = 0xFFFFF },
	[RI_IFCR] = {.mask = 0xFFFFF },
	CHAN_REGINFO(1),
	CHAN_REGINFO(2),
	CHAN_REGINFO(3),
	CHAN_REGINFO(4),
	CHAN_REGINFO(5),
	CHAN_RESERVED(6),
	CHAN_RESERVED(7),
};

static const stm32_reginfo_t stm32g070_dma_reginfo[RI_END] =
{
	[RI_ISR] = {.mask = 0x0FFFFFFF },
	[RI_IFCR] = {.mask = 0x0FFFFFFF },
	[RI_CHAN_BASE+CH_OFF_CCR] = {.mask = 0x7FFF },
	[RI_CHAN_BASE+CH_OFF_CNDTR] = {.mask = UINT16_MAX },
	CHAN_REGINFO(1),
	CHAN_REGINFO(2),
	CHAN_REGINFO(3),
	CHAN_REGINFO(4),
	CHAN_REGINFO(5),
	CHAN_REGINFO(6),
	CHAN_REGINFO(7),
};

 static const stm32_periph_variant_t stm32_dma_variants[2] = {
 	{TYPE_STM32F030_DMA, stm32f030_dma_reginfo},
 	{TYPE_STM32G070_DMA, stm32g070_dma_reginfo},
 };

static void stm32_common_update_irqs(COM_STRUCT_NAME(Dma) *s, uint8_t channel)
{
	uint32_t sr = s->regs.defs.ISR.raw;
	uint32_t gi_flag = INT_GI << (channel*INT_BLKSIZE);

	uint32_t int_en = s->regs.raw[(RI_CHAN_BASE+(STM32_COM_DMA_CHAN_REGS*channel)+CH_OFF_CCR)];
	int_en &= INT_ALL;

	sr>>=(channel*INT_BLKSIZE);
	sr &= INT_ALL;
	if (sr != 0)
	{
		s->regs.defs.ISR.raw |= gi_flag;
		if (sr & int_en) {
			qemu_irq_raise(s->irq[channel]);
		}
	}
	else
	{
		s->regs.defs.ISR.raw &= ~gi_flag;
		qemu_irq_lower(s->irq[channel]);
	}
}

static void stm32_common_update_all_irqs(COM_STRUCT_NAME(Dma) *s)
{
	for (int i=0; i< STM32_COM_DMA_MAX_CHAN; i++)
	{
		stm32_common_update_irqs(s, i);
	}
}

static void stm32_common_set_int_flag(COM_STRUCT_NAME(Dma) *s, uint8_t channel, uint32_t flag)
{
	flag <<= (channel*INT_BLKSIZE);
	uint32_t uiChanged = s->regs.defs.ISR.raw ^ flag;
	s->regs.defs.ISR.raw |= flag;
	if (uiChanged)
	{
		stm32_common_update_irqs(s, channel);
	}
}

static void stm32_common_dma_do_xfer(COM_STRUCT_NAME(Dma) *s, hwaddr chan_base)
{
	uint8_t channel = (chan_base - RI_CHAN_BASE) / STM32_COM_DMA_CHAN_REGS;
	REGDEF_NAME(dma,ccr)* cr = (REGDEF_NAME(dma,ccr)*)&s->regs.raw[chan_base+CH_OFF_CCR];
	uint8_t dir = cr->DIR;
	uint32_t* dest = NULL;
	uint32_t* src = NULL;
	uint8_t dest_inc = 0, src_inc = 0;
	uint8_t dest_size = 0, src_size  = 0;
	if (dir == DIR_P2M)
	{
		dest = &s->regs.raw[chan_base+CH_OFF_CMAR];
		dest_size = dma_xfer_size_b[cr->MSIZE];
		dest_inc = cr->MINC * dest_size;
		src = &s->regs.raw[chan_base+CH_OFF_CPAR];
		src_size = dma_xfer_size_b[cr->PSIZE];
		src_inc = cr->PINC * src_size;
	}
	else
	{
		src = &s->regs.raw[chan_base+CH_OFF_CMAR];
		src_size = dma_xfer_size_b[cr->MSIZE];
		src_inc = cr->MINC * src_size;
		dest = &s->regs.raw[chan_base+CH_OFF_CPAR];
		dest_size = dma_xfer_size_b[cr->PSIZE];
		dest_inc = cr->PINC * dest_size;
	}
	uint32_t *ndtr = &s->regs.raw[chan_base+CH_OFF_CNDTR];
	// printf("DMA Transfer: 0x%" PRIx32 "->0x%" PRIx32 ", size %u ndtr %u ",*src, *dest, xfersize, *ndtr);
	uint8_t buff[4] = {0,0,0,0};
	dma_memory_read(
		&s->cpu_as,
		*src,
		buff,
		src_size,
		MEMTXATTRS_UNSPECIFIED
	);
	// printf("d: %02x\n", buff[0]);
	dma_memory_write(
		&s->cpu_as,
		*dest,
		buff,
		dest_size,
		MEMTXATTRS_UNSPECIFIED
	);

	*dest += dest_inc;
	*src += src_inc;

	(*ndtr)--; // NDTR is in transfers, not bytes.

	if (*ndtr == (s->original_ndtrs[channel]>>1))
	{
		stm32_common_set_int_flag(s, channel,INT_HT);
	}
	else if (*ndtr == 0 )
	{
		stm32_common_set_int_flag(s, channel,INT_TC);
		if (cr->CIRC)
		{
			*ndtr = s->original_ndtrs[channel];
			*dest -= (*ndtr*dest_inc);
			*src -= (*ndtr*src_inc);
		}
		else
		{
			cr->EN = false; // Disable the channel, transmit is done.
		}
	}
}

static void stm32_common_dma_dmar(void *opaque, int n, int level)
{
	//Idea: level contains the source address of the peripheral,
	// so that we don't have to look up a DR by peripheral ID like
	// in the STM32F4 implementation.
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(opaque);

	REGDEF_NAME(dma,ccr) *cr;
	int idx = 0;
	for (int i=RI_CHAN_BASE; i<RI_CHAN_END; i+=STM32_COM_DMA_CHAN_REGS, idx++)
	{
		cr = (REGDEF_NAME(dma,ccr)*)&s->regs.raw[i+CH_OFF_CCR];
		// Is this our peripheral?
		if (s->regs.raw[i+CH_OFF_CPAR] != level)
		{
			continue;
		}
		if (cr->EN != true || s->regs.raw[i+CH_OFF_CNDTR] == 0)
		{
			continue;
		}
		if (cr->DIR != n) // Does the direction of the request match?
		{
			continue;
		}
		s->dma_pending[idx]++;
		// Rather than immediate DMA, we use a timer and channel pending count
		// to keep things from going pear-shaped if the DMA action triggers
		// a cascade - the triggered event will be delayed until the timer fires
		// giving the current transaction a chance to finish.
		timer_mod_ns(s->dma_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
		//stm32_common_dma_do_xfer(s, i);
	}
}

static void stm32_common_dma_timer(void *opaque)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(opaque);
	for (int i=STM32_COM_DMA_MAX_CHAN - 1U; i>=0; i--)
	{
		if (s->dma_pending[i])
		{
			stm32_common_dma_do_xfer(s,RI_CHAN_BASE+(i*STM32_COM_DMA_CHAN_REGS));
			s->dma_pending[i]--;
		}
	}
}

static uint64_t
stm32_common_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(opaque);
    int offset = addr & 0x3;

    addr >>= 2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32 Common DMA"); // LCOV_EXCL_LINE

	// NOTE - IFCR is read-only, but we never change its contents in the write so it will always come back 0
    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b100);
    return value;
}

static void
stm32_common_dma_chan_write(COM_STRUCT_NAME(Dma) *s, hwaddr addr, uint64_t data, unsigned int size)
{
	uint8_t chan = (addr - RI_CHAN_BASE)/STM32_COM_DMA_CHAN_REGS;
	uint8_t offset = (addr - RI_CHAN_BASE)%STM32_COM_DMA_CHAN_REGS;
	// printf("Wrote DMA channel %u/off %u with value %0"PRIx64"\n", chan, offset, data);
	switch (offset)
	{
		case CH_OFF_CCR:
		{
			REGDEF_NAME(dma,ccr) old = {.raw = s->regs.raw[addr]};
			REGDEF_NAME(dma,ccr) new = {.raw = data};
			s->regs.raw[addr] = data;
			if (!old.EN && new.EN)
			{
				// printf("Enabled DMA channel %u\n", chan);
				// Channel was enabled... if P2M we wait for P to signal DMAR anyway.
			}
			else if(!new.EN && old.EN)
			{
				// DMA was disabled. Reset CMAR.
				s->regs.raw[addr+CH_OFF_CMAR] = s->original_cmars[chan];
				s->regs.raw[addr+CH_OFF_CPAR] = s->original_cpars[chan];
			}
		}
		break;
		case CH_OFF_CNDTR:
			s->original_ndtrs[chan] = data & 0xFFFFU;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_CPAR:
			s->original_cpars[chan] = data;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_CMAR:
			s->original_cmars[chan] = data;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_END: // LCOV_EXCL_LINE
			qemu_log_mask(LOG_GUEST_ERROR, "ERR: DMA write to reserved register!"); //LCOV_EXCL_LINE
	}
}

static void
stm32_common_dma_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(opaque);

    int offset = addr & 0x3;

    addr >>= 2;

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 6);

	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32 Common DMA"); // LCOV_EXCL_LINE
	CHECK_UNIMP_RESVD(data, s->reginfo, addr);

    switch(addr) {
    case RI_ISR:
		qemu_log_mask(LOG_GUEST_ERROR, "ERR: Attempted to write read-only STM32 DMA ISR register.");
        break;
	case RI_IFCR:
		// Note: GIF clearing is not implemented...
		s->regs.defs.ISR.raw &= ~(data);
		stm32_common_update_all_irqs(s);
		break;
	case RI_CHAN_BASE ... RI_CHAN_END:
		stm32_common_dma_chan_write(s, addr, data, size);
        break;
    default: // LCOV_EXCL_LINE
        qemu_log_mask(LOG_UNIMP, "stm32common_dma unimplemented write 0x%x+%u size %u val 0x%x\n", // LCOV_EXCL_LINE
        (unsigned int)addr << 2, offset, size, (unsigned int)data); // LCOV_EXCL_LINE
    }
}

static const MemoryRegionOps stm32_common_dma_ops = {
    .read = stm32_common_dma_read,
    .write = stm32_common_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};

static void stm32_common_dma_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(dev);
    memset(&s->regs, 0, sizeof(s->regs));
}

static void stm32_common_dma_realize(DeviceState *dev, Error **errp)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(dev);
	if (s->cpu_mr == NULL)
	{
		qemu_log_mask(LOG_UNIMP, "No CPU memory region specified for %s - using global system memory.\n", _PERIPHNAMES[s->parent.periph]);
		s->cpu_mr = get_system_memory();
	}
	gchar* name = g_strdup_printf("STM32COM_DMA_%d", s->parent.periph - STM32_P_DMA_BEGIN);
	address_space_init(&s->cpu_as, s->cpu_mr, name);
	g_free(name);
}

static void
stm32_common_dma_init(Object *obj)
{
	COM_STRUCT_NAME(Dma) *s = STM32COM_DMA(obj);


    QEMU_BUILD_BUG_MSG(sizeof(s->regs.defs)!=sizeof(s->regs.raw), __FILE__" definitions/unions do not align!"); // Make sure packing is correct.
    // CHECK_REG_u32(s->regs.defs.ISR);
    CHECK_REG_u32(s->regs.defs.IFCR);
	CHECK_REG_u32(s->regs.defs.CCR1);
	CHECK_REG_u32(s->regs.defs.CNDTR1);
	CHECK_REG_u32(s->regs.defs.CPAR1);
	CHECK_REG_u32(s->regs.defs.CMAR1);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, interrupt), defs.ISR);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, interrupt), defs.IFCR);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, ccr), defs.CCR);


    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_dma_ops, s, 1*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in_named(DEVICE(obj),stm32_common_dma_dmar,"dmar-in",2);

	qdev_init_gpio_out_named(DEVICE(obj), s->irq, SYSBUS_DEVICE_GPIO_IRQ, STM32_COM_DMA_MAX_CHAN);

	COM_CLASS_NAME(Dma) *k = STM32COM_DMA_GET_CLASS(obj);

	s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_common_dma_timer, obj);

	s->reginfo = k->var_reginfo;
}

static const VMStateDescription vmstate_stm32_common_dma = {
    .name = TYPE_STM32COM_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,COM_STRUCT_NAME(Dma), RI_END),
        VMSTATE_UINT32_ARRAY(original_ndtrs,COM_STRUCT_NAME(Dma), STM32_COM_DMA_MAX_CHAN),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32_common_dma_properties[] = {
    DEFINE_PROP_LINK("system-memory", COM_STRUCT_NAME(Dma), cpu_mr, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_common_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_common_dma;
    dc->reset = stm32_common_dma_reset;
	dc->realize = stm32_common_dma_realize;
	device_class_set_props(dc, stm32_common_dma_properties);

	COM_CLASS_NAME(Dma) *k = STM32COM_DMA_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
};

static const TypeInfo
stm32_common_dma_info = {
    .name          = TYPE_STM32COM_DMA,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Dma)),
	.class_size    = sizeof(COM_CLASS_NAME(Dma)),
};

static void
stm32_common_dma_register_types(void)
{
    type_register_static(&stm32_common_dma_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_dma_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_dma_variants[i].variant_name,
            .parent     = TYPE_STM32COM_DMA,
			.instance_init = stm32_common_dma_init,
			.class_init    = stm32_common_dma_class_init,
            .class_data = (void *)stm32_dma_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_dma_register_types)
