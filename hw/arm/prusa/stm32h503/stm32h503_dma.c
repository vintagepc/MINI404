/*
    stm32h503_dma.c - DMA for STM32H503.

	Copyright 2024 VintagePC <https://github.com/vintagepc/>

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
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_shared.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "../stm32_common/stm32_rcc_if.h"

#include "../stm32_registers/generated/stm32h503/DMA_index.h"
#include "../stm32_registers/generated/stm32h503/DMA_registers.h"
#include "../stm32_registers/generated/stm32h503/Addresses.h"

#define DMA_STRIDE 0x80
#define DMA_NUM_CHANS (8U)

#define DEBUG 0

#if DEBUG
#define DBGPRINTF(...) printf(__VA_ARGS__)
#else
#define DBGPRINTF(...)
#endif

enum stm32h503_dma_blocks {
	RI_CHAN_BASE = RI_CLBAR,
	RI_CHAN0 = RI_CHAN_BASE,
	RI_CHAN1 = 		RI_CHAN0 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN2 = 		RI_CHAN1 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN3 = 		RI_CHAN2 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN4 = 		RI_CHAN3 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN5 = 		RI_CHAN4 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN6 = 		RI_CHAN5 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN7 = 		RI_CHAN6 + (DMA_STRIDE/sizeof(uint32_t)),
	RI_CHAN_END = 	RI_CHAN7 + (DMA_STRIDE/sizeof(uint32_t)),
};

enum stm32h503_dma_offsets {
	CH_OFF_CLBAR = RI_CLBAR - RI_CHAN_BASE,
	CH_OFF_CFCR = RI_CFCR - RI_CHAN_BASE,
	CH_OFF_CSR = RI_CSR - RI_CHAN_BASE,
	CH_OFF_CCR = RI_CCR - RI_CHAN_BASE,
	CH_OFF_CTR1 = RI_CTR1 - RI_CHAN_BASE,
	CH_OFF_CTR2 = RI_CTR2 - RI_CHAN_BASE,
	CH_OFF_CBR1 = RI_CBR1 - RI_CHAN_BASE,
	CH_OFF_CSAR = RI_CSAR - RI_CHAN_BASE,
	CH_OFF_CDAR = RI_CDAR - RI_CHAN_BASE,
	CH_OFF_CTR3 = RI_CTR3 - RI_CHAN_BASE,
	CH_OFF_CBR2 = RI_CBR2 - RI_CHAN_BASE,
	CH_OFF_CLLR = RI_CLLR - RI_CHAN_BASE,
};

OBJECT_DECLARE_SIMPLE_TYPE(STM32H503_STRUCT_NAME(Dma), STM32H503_DMA);

typedef struct REGDEF_NAME(dma, block) {
	REGDEF_NAME(h503_dma,clbar) CLBAR;
	uint32_t _reserved0x54[ 2];
	REGDEF_NAME(h503_dma,cfcr) CFCR;
	REGDEF_NAME(h503_dma,csr) CSR;
	REGDEF_NAME(h503_dma,ccr) CCR;
	uint32_t _reserved0x68[10];
	REGDEF_NAME(h503_dma,ctr1) CTR1;
	REGDEF_NAME(h503_dma,ctr2) CTR2;
	REGDEF_NAME(h503_dma,cbr1) CBR1;
	REGDEF_NAME(h503_dma,csar) CSAR;
	REGDEF_NAME(h503_dma,cdar) CDAR;
	REGDEF_NAME(h503_dma,ctr3) CTR3;
	REGDEF_NAME(h503_dma,cbr2) CBR2;
	uint32_t _reserved0xAC[ 8];
	REGDEF_NAME(h503_dma,cllr) CLLR;
} REGDEF_NAME(dma, block);
QEMU_BUILD_BUG_MSG(sizeof(REGDEF_NAME(dma, block)) != DMA_STRIDE, "DMA block size mismatch!");

typedef struct STM32H503_STRUCT_NAME(Dma) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

	MemoryRegion* cpu_mr;
	AddressSpace cpu_as;

    union {
        struct {
			uint32_t _reserved0x0;
			REGDEF_NAME(h503_dma,privcfgr) PRIVCFGR;
			uint32_t _reserved0x8;
			REGDEF_NAME(h503_dma,misr) MISR;
			uint32_t _reserved0x10[16];
			REGDEF_NAME(dma, block) blocks[DMA_NUM_CHANS];
		};
        uint32_t raw[RI_CHAN_END];
    } regs;

	uint32_t original_ndtrs[DMA_NUM_CHANS];
	uint32_t original_sars[DMA_NUM_CHANS];
	uint32_t original_dars[DMA_NUM_CHANS];

	QEMUTimer* dma_timer;
	uint8_t dma_pending[DMA_NUM_CHANS];

	qemu_irq irq[DMA_NUM_CHANS];

	const stm32_reginfo_t reginfo[RI_CHAN_END];

} STM32H503_STRUCT_NAME(Dma);

#undef _STM32_DMA_CHAN_BLK

enum dma_dir
{
	DIR_P2M,
	DIR_M2P,
};

enum interrupt_bits
{
	INT_IDLE = BIT(0),
	INT_TC = BIT(8),
	INT_HT = BIT(9),
	INT_TE = BIT(10),
	INT_ULE = BIT(11),
	INT_USRE = BIT(12),
	INT_SUSPF = BIT(13),
	INT_TOVRF = BIT(14),
};

static const uint8_t dma_xfer_size_b[4] = {1, 2, 4, 0};

typedef struct COM_CLASS_NAME(Dma) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_CHAN_END];
} COM_CLASS_NAME(Dma);


//  static const stm32_periph_variant_t stm32_dma_variants[1] = {
//  	{TYPE_STM32H503_DMA, stm32_h503_dma_reginfo},
//  };

static void stm32_common_update_irqs(STM32H503_STRUCT_NAME(Dma) *s, uint8_t channel)
{

	uint32_t chan_int = s->regs.blocks[channel].CSR.raw & s->regs.blocks[channel].CCR.raw;

	bool new_int = chan_int>0;
	if (new_int)
	{
		s->regs.MISR.raw |= BIT(channel);
		qemu_irq_raise(s->irq[channel]);
		DBGPRINTF("DMA IRQ %u raised\n", channel);
	}
	else
	{
		s->regs.MISR.raw &= ~BIT(channel);
		qemu_irq_lower(s->irq[channel]);
		DBGPRINTF("DMA IRQ %u lowered\n", channel);
	}
}

// static void stm32_common_update_all_irqs(STM32H503_STRUCT_NAME(Dma) *s)
// {
// 	for (int i=0; i< DMA_NUM_CHANS; i++)
// 	{
// 		stm32_common_update_irqs(s, i);
// 	}
// }

static void stm32_common_set_int_flag(STM32H503_STRUCT_NAME(Dma) *s, uint8_t channel, uint32_t flag)
{
	uint32_t uiChanged = s->regs.blocks[channel].CSR.raw ^ flag;
	if (uiChanged)
	{
		s->regs.blocks[channel].CSR.raw |= flag;
		stm32_common_update_irqs(s, channel);
	}
}

static bool stm32_h503_dma_is_m2m(STM32H503_STRUCT_NAME(Dma) *s, int channel)
{
	return s->regs.blocks[channel].CSAR.bits.SA < H503_PERIPH_ADDR &&
		s->regs.blocks[channel].CDAR.bits.DA < H503_PERIPH_ADDR;
}

static void stm32_h503_dma_do_xfer(STM32H503_STRUCT_NAME(Dma) *s, int channel)
{
	stm32reg_dma_block_t* block = &s->regs.blocks[channel];
	uint8_t dest_inc = block->CTR1.bits.DINC;
	uint8_t src_inc = block->CTR1.bits.SINC;
	uint8_t dest_size = dma_xfer_size_b[block->CTR1.bits.DDW_LOG2];
	uint8_t src_size = dma_xfer_size_b[block->CTR1.bits.SDW_LOG2];

	uint32_t *ndtr = &block->CBR1.raw;
	DBGPRINTF("DMA Transfer: 0x%" PRIx32 "->0x%" PRIx32 ", size %u ndtr %u\n",block->CSAR.bits.SA, block->CDAR.bits.DA, src_size, *ndtr);
	uint8_t buff[4] = {0,0,0,0};
	dma_memory_read(
		&s->cpu_as,
		block->CSAR.raw,
		buff,
		src_size,
		MEMTXATTRS_UNSPECIFIED
	);
	// printf("d: %02x\n", buff[0]);
	dma_memory_write(
		&s->cpu_as,
		block->CDAR.raw,
		buff,
		dest_size,
		MEMTXATTRS_UNSPECIFIED
	);

	block->CDAR.bits.DA += dest_inc;
	block->CSAR.bits.SA += src_inc;

	(*ndtr)--; // NDTR is in transfers, not bytes.

	if (*ndtr == (s->original_ndtrs[channel]>>1))
	{
		stm32_common_set_int_flag(s, channel,INT_HT);
		DBGPRINTF("Half transfer\n");
	}
	else if (*ndtr == 0 )
	{
		block->CCR.bits.EN = false; // Disable the channel, transmit is done.
		stm32_common_set_int_flag(s, channel,INT_TC);
		DBGPRINTF("Transfer complete\n");
	}

	if (*ndtr > 0 && stm32_h503_dma_is_m2m(s, channel))
	{
		// This is an M2M transfer, so keep triggering it.
		s->dma_pending[channel]++;
		timer_mod_ns(s->dma_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
	}
}

static void stm32_h503_dma_dmar(void *opaque, int n, int level)
{
	//Idea: level contains the source address of the peripheral,
	// so that we don't have to look up a DR by peripheral ID like
	// in the STM32F4 implementation.
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(opaque);
	int idx = 0;
	for (int i=0; i<DMA_NUM_CHANS; i++)
	{
		// Is this our peripheral?
		if ((s->regs.blocks[i].CSAR.bits.SA != level && n != DIR_P2M) ||
			(s->regs.blocks[i].CDAR.bits.DA != level && n != DIR_M2P))
		{
			continue;
		}
		if (s->regs.blocks[i].CCR.bits.EN != true)
		{
			continue;
		}
		s->dma_pending[idx]++;
		// Rather than immediate DMA, we use a timer and channel pending count
		// to keep things from going pear-shaped if the DMA action triggers
		// a cascade - the triggered event will be delayed until the timer fires
		// giving the current transaction a chance to finish.
		timer_mod_ns(s->dma_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
		//stm32_h503_dma_do_xfer(s, i);
	}
}

static void stm32_h503_dma_timer(void *opaque)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(opaque);
	for (int i=DMA_NUM_CHANS - 1U; i>=0; i--)
	{
		if (s->dma_pending[i])
		{
			stm32_h503_dma_do_xfer(s, i);
			s->dma_pending[i]--;
		}
	}
}

static uint64_t
stm32_h503_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(opaque);
    int offset = addr & 0x3;

    addr >>= 2;

	CHECK_BOUNDS_R_V2(addr, RI_CHAN_END, s->reginfo); // LCOV_EXCL_LINE

	// NOTE - IFCR is read-only, but we never change its contents in the write so it will always come back 0
    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b100);
    return value;
}

static void
stm32_h503_dma_chan_write(STM32H503_STRUCT_NAME(Dma) *s, hwaddr addr, uint64_t data, unsigned int size)
{
	uint8_t chan =   (addr - RI_CHAN_BASE)/(DMA_STRIDE/sizeof(uint32_t));
	uint8_t offset = (addr - RI_CHAN_BASE)%(DMA_STRIDE/sizeof(uint32_t));
	DBGPRINTF("Wrote DMA channel %u/off %u with value %0"PRIx64"\n", chan, offset, data);
	switch (offset)
	{
		case CH_OFF_CCR:
		{
			REGDEF_NAME(h503_dma,ccr) new = {.raw = data};
			if (!s->regs.blocks[chan].CCR.bits.EN && new.bits.EN)
			{
				s->regs.blocks[chan].CSR.bits.IDLEF = false;
				if (stm32_h503_dma_is_m2m(s, chan))
				{
					// This is an M2[something] transfer, so start it.
					s->dma_pending[chan]++;
					timer_mod_ns(s->dma_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
				}
			}
			else if(!new.bits.EN && s->regs.blocks[chan].CCR.bits.EN)
			{
				// DMA was disabled. Reset CMAR.
				s->regs.blocks[chan].CSAR.raw = s->original_sars[chan];
				s->regs.blocks[chan].CDAR.raw = s->original_dars[chan];
				s->regs.blocks[chan].CSR.bits.IDLEF = true;
			}
			stm32_common_update_irqs(s, chan);
		}
		break;
		case CH_OFF_CFCR:
			s->regs.blocks[chan].CSR.raw &= ~data;
			return stm32_common_update_irqs(s, chan);
			break;
		case CH_OFF_CBR1:
			s->original_ndtrs[chan] = data & 0xFFFFU;
			break;
		case CH_OFF_CSAR:
			break;
		case CH_OFF_CDAR:
			break;
	}
	s->regs.raw[addr] = data;
}

static void
stm32_h503_dma_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(opaque);

    int offset = addr & 0x3;

    addr >>= 2;

	CHECK_BOUNDS_W_V2(addr, data, RI_CHAN_END); // LCOV_EXCL_LINE
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 6);
	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[addr], s->reginfo, addr);

    switch(addr) {
    case RI_MISR:
		qemu_log_mask(LOG_GUEST_ERROR, "ERR: Attempted to write read-only STM32 DMA ISR register.");
        break;
	case RI_CHAN_BASE ... RI_CHAN_END:
		stm32_h503_dma_chan_write(s, addr, data, size);
        break;
    }
}

static const MemoryRegionOps stm32_h503_dma_ops = {
    .read = stm32_h503_dma_read,
    .write = stm32_h503_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};

static void stm32_h503_dma_reset(DeviceState *dev)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(dev);
    memset(&s->regs, 0, sizeof(s->regs));
}

static void stm32_h503_dma_realize(DeviceState *dev, Error **errp)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(dev);
	if (s->cpu_mr == NULL)
	{
		qemu_log_mask(LOG_UNIMP, "No CPU memory region specified for %s - using global system memory.\n", _PERIPHNAMES[s->parent.periph]);
		s->cpu_mr = get_system_memory();
	}
	gchar* name = g_strdup_printf("STM32H503_DMA_%d", s->parent.periph - STM32_P_DMA_BEGIN);
	address_space_init(&s->cpu_as, s->cpu_mr, name);
	g_free(name);
}

static void
stm32_h503_dma_init(Object *obj)
{
	STM32H503_STRUCT_NAME(Dma) *s = STM32H503_DMA(obj);

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_h503_dma_ops, s, 1*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in_named(DEVICE(obj),stm32_h503_dma_dmar,"dmar-in",2);

	qdev_init_gpio_out_named(DEVICE(obj), s->irq, SYSBUS_DEVICE_GPIO_IRQ, DMA_NUM_CHANS);

	s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_h503_dma_timer, obj);

	// Copy the generated info to construct the "full" register table.
	stm32_reginfo_t* rdest = (stm32_reginfo_t*)&s->reginfo;
	memcpy(&rdest[RI_PRIVCFGR], &stm32_h503_dma_reginfo[RI_PRIVCFGR], sizeof(stm32_reginfo_t));
	memcpy(&rdest[RI_MISR],  	&stm32_h503_dma_reginfo[RI_MISR], sizeof(stm32_reginfo_t));
	memcpy(&rdest[RI_CHAN0], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN1], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN2], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN3], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN4], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN5], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN6], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	memcpy(&rdest[RI_CHAN7], 	&stm32_h503_dma_reginfo[RI_CHAN_BASE], sizeof(stm32_reginfo_t)*DMA_STRIDE/sizeof(uint32_t));
	QEMU_BUILD_BUG_MSG(sizeof(s->reginfo) != sizeof(stm32_reginfo_t[RI_CHAN_END]), "Reginfo not sized correctly!");
}

static const VMStateDescription vmstate_stm32_h503_dma = {
    .name = TYPE_STM32H503_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32H503_STRUCT_NAME(Dma), RI_CHAN_END),
        VMSTATE_UINT32_ARRAY(original_ndtrs,STM32H503_STRUCT_NAME(Dma), DMA_NUM_CHANS),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32_h503_dma_properties[] = {
    DEFINE_PROP_LINK("system-memory", STM32H503_STRUCT_NAME(Dma), cpu_mr, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_h503_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_h503_dma;
	device_class_set_legacy_reset(dc, stm32_h503_dma_reset);
	dc->realize = stm32_h503_dma_realize;
	device_class_set_props(dc, stm32_h503_dma_properties);
};


static TypeInfo stm32_h503_dma_info = {
    .name  = TYPE_STM32H503_DMA,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size  = sizeof(STM32H503_STRUCT_NAME(Dma)),
    .class_init = stm32_h503_dma_class_init,
    .instance_init = stm32_h503_dma_init,
};

static void stm32_h503_dma_register_types(void)
{
    type_register_static(&stm32_h503_dma_info);
}

type_init(stm32_h503_dma_register_types)
