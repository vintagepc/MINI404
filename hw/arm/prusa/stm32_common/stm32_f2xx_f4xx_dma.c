/*
    stm32_dma.h - DMA for STM32F2/4.
	So far the DMA layout is common to the following chips:
	- F4xx series,
	- F2xx series (? - it's modelled off the Pebble QEMU version which supports this.),

	Copyright 2021-2 VintagePC <https://github.com/vintagepc/>

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
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "stm32_f2xx_f4xx_dma_regdata.h"


OBJECT_DECLARE_SIMPLE_TYPE(STM32F2XX_STRUCT_NAME(Dma), STM32F4xx_DMA);

#define _STM32_DMA_INT_BITSET(chan) \
	REG_B32(_JOIN2R(FEIF,chan)); \
	REG_R(1); \
	REG_B32(_JOIN2R(DMEIF,chan)); \
	REG_B32(_JOIN2R(TEIF,chan)); \
	REG_B32(_JOIN2R(HTIF,chan)); \
	REG_B32(_JOIN2R(TCIF,chan));


REGDEF_BLOCK_BEGIN()
	_STM32_DMA_INT_BITSET(0)
	_STM32_DMA_INT_BITSET(1)
	REG_R(4);
	_STM32_DMA_INT_BITSET(2)
	_STM32_DMA_INT_BITSET(3)
	REG_R(4);
REGDEF_BLOCK_END(dma, low_int)

REGDEF_BLOCK_BEGIN()
	_STM32_DMA_INT_BITSET(4)
	_STM32_DMA_INT_BITSET(5)
	REG_R(4);
	_STM32_DMA_INT_BITSET(6)
	_STM32_DMA_INT_BITSET(7)
	REG_R(4);
REGDEF_BLOCK_END(dma, high_int)

#undef _STM32_DMA_INT_BITSET

REGDEF_BLOCK_BEGIN()
	REG_B32(EN);
	REG_B32(DMEIE);
	REG_B32(TEIE);
	REG_B32(HTIE);
	REG_B32(TCIE);
	REG_B32(PFCTRL);
	REG_K32(DIR,2);
	REG_B32(CIRC);
	REG_B32(PINC);
	REG_B32(MINC);
	REG_K32(PSIZE,2);
	REG_K32(MSIZE,2);
	REG_B32(PINCOS);
	REG_K32(PL,2);
	REG_B32(DBM);
	REG_B32(CT);
	REG_R(1);
	REG_K32(PBURST,2);
	REG_K32(MBURST,2);
	REG_K32(CHSEL,3);
	REG_R(4);
REGDEF_BLOCK_END(dma, sxcr)

REGDEF_BLOCK_BEGIN()
	REG_K32(FTH,2);
	REG_B32(DMDIS);
	REG_K32(FS,3);
	REG_R(1);
	REG_B32(FEIE);
	REG_R(24);
REGDEF_BLOCK_END(dma, sxfcr)

#define _STM32_DMA_CHAN_BLK(x) \
	REGDEF_NAME(dma, sxcr) _JOIN2R(SCR,x); \
	REG_S32(NDTR, 16) _JOIN2R(SNDTR,x); \
	uint32_t _JOIN2R(SPAR,x); \
	uint32_t _JOIN2R(SM0AR,x); \
	uint32_t _JOIN2R(SM1AR,x); \
	REGDEF_NAME(dma, sxfcr) _JOIN2R(SFCR,x);



typedef struct STM32F2XX_STRUCT_NAME(Dma) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

	MemoryRegion* cpu_mr;
	AddressSpace cpu_as;

    union {
        struct {
			REGDEF_NAME(dma, low_int) LISR;			//0x00
			REGDEF_NAME(dma, high_int) HISR;		//0x04
			REGDEF_NAME(dma, low_int) LIFCR;		//0x08
			REGDEF_NAME(dma, high_int) HIFCR;		//0x0C
			_STM32_DMA_CHAN_BLK(0)	 				//0x10 - 0x24
			_STM32_DMA_CHAN_BLK(1)	 				//0x28 - 0x3C
			_STM32_DMA_CHAN_BLK(2)	 				//0x40 - 0x54
			_STM32_DMA_CHAN_BLK(3)	 				//0x58 - 0x6C
			_STM32_DMA_CHAN_BLK(4)	 				//0x70 - 0x84
			_STM32_DMA_CHAN_BLK(5)	 				//0x88 - 0x9C
			_STM32_DMA_CHAN_BLK(6)	 				//0xA0 - 0xB4
			_STM32_DMA_CHAN_BLK(7)	 				//0xB8 - 0xCC
		} defs;
        uint32_t raw[RI_MAX];
    } regs;

	uint32_t original_ndtrs[STM32_F2xx_DMA_MAX_CHAN];
	uint32_t original_cmars[STM32_F2xx_DMA_MAX_CHAN];
	uint32_t original_cpars[STM32_F2xx_DMA_MAX_CHAN];
	uint8_t dma_pending[STM32_F2xx_DMA_MAX_CHAN];

	QEMUTimer* dma_timer;

	qemu_irq irq[STM32_F2xx_DMA_MAX_CHAN];

} STM32F2XX_STRUCT_NAME(Dma);


enum dma_dir
{
	DIR_P2M, // NOTE - value must match DMAR_P2M
	DIR_M2P, // and this must equal DMAR_M2P for proper IRQ handling.
	DIR_M2M,
};

enum interrupt_bits
{
	INT_FEIF = 	0b000001,
	INT_DMEIF = 0b000100,
	INT_TEIF = 	0b001000,
	INT_HTIF = 	0b010000,
	INT_TCIF = 	0b100000,
	INT_ALL = 	0b111101,
	INT_SxCR_MASK = 0b111100,
	INT_BLKSIZE = 6,
};

static const uint8_t dma_xfer_size_b[4] = {1, 2, 4, 0};

QEMU_BUILD_BUG_MSG(CH_OFF_END != STM32_F2xx_DMA_CHAN_REGS, "register definitions do not agree!");


static void stm32_f2xx_f4xx_update_irqs(STM32F2XX_STRUCT_NAME(Dma) *s, uint8_t channel)
{
	// uint32_t sr = s->regs.defs.ISR.raw;

	uint32_t sxcr = s->regs.raw[(RI_CHAN_BASE+(channel*STM32_F2xx_DMA_CHAN_REGS)+CH_OFF_SxCR)];
	REGDEF_NAME(dma, sxfcr) sxfcr = {.raw = s->regs.raw[(RI_CHAN_BASE+(channel*STM32_F2xx_DMA_CHAN_REGS)+CH_OFF_SxFCR)]};
	// reconstruct enable mask for ISR
	uint32_t int_en = ((sxcr & INT_SxCR_MASK) << 1U) | sxfcr.FEIE;

	uint32_t sr = 0;
	if (channel > 3U)
	{
		sr = s->regs.defs.HISR.raw;
	}
	else
	{
		sr = s->regs.defs.LISR.raw;
	}
	if (channel % 4 > 1U) // 2-3, 6-7 are upper 8 bits
	{
		sr >>= 16U;
	}
	if (channel % 2) // 1, 3, 5, 7 are upper 8 bits
	{
		sr >>= 6U;
	}

	sr &= INT_ALL;

	if (sr != 0)
	{
		if (sr & int_en) {
			qemu_irq_raise(s->irq[channel]);
		}
	}
	else
	{
		qemu_irq_lower(s->irq[channel]);
	}
}

static void stm32_f2xx_f4xx_update_all_irqs(STM32F2XX_STRUCT_NAME(Dma) *s)
{
	for (int i=0; i< STM32_F2xx_DMA_MAX_CHAN; i++)
	{
		stm32_f2xx_f4xx_update_irqs(s, i);
	}
}

static void stm32_f2xx_f4xx_set_int_flag(STM32F2XX_STRUCT_NAME(Dma) *s, uint8_t channel, uint32_t flag)
{
	if (channel % 4 > 1U) // 2-3, 6-7 are upper 8 bits
	{
		flag <<= 16U;
	}
	if (channel % 2) // 1, 3, 5, 7 are upper 8 bits
	{
		flag <<= 6U;
	}
	uint32_t *src = NULL;
	if (channel > 3U)
	{
		src = &s->regs.defs.HISR.raw;
	}
	else
	{
		src = &s->regs.defs.LISR.raw;
	}

	uint32_t changed = ((*src) ^ flag) & flag ;
	(*src) |= flag;
	if (changed)
	{
		stm32_f2xx_f4xx_update_irqs(s, channel);
	}
}

static void stm32_f2xx_f4xx_dma_do_xfer(STM32F2XX_STRUCT_NAME(Dma) *s, hwaddr chan_base)
{
	uint8_t channel = (chan_base - RI_CHAN_BASE) / STM32_F2xx_DMA_CHAN_REGS;
	REGDEF_NAME(dma,sxcr)* cr = (REGDEF_NAME(dma,sxcr)*)&s->regs.raw[chan_base+CH_OFF_SxCR];
	uint8_t dir = cr->DIR;
	uint32_t *ndtr = &s->regs.raw[chan_base+CH_OFF_SxNDTR];
	uint32_t *dest = NULL;
	uint32_t *src = NULL;
	uint8_t src_size = 0, dest_size = 0;
	uint8_t dest_inc = 0, src_inc = 0;
	if (dir == DIR_P2M)
	{
		dest = &s->regs.raw[chan_base+CH_OFF_SxM0AR];
		dest_size = dma_xfer_size_b[cr->MSIZE];
		dest_inc = cr->MINC * dest_size;
		src = &s->regs.raw[chan_base+CH_OFF_SxPAR];
		src_size = dma_xfer_size_b[cr->PSIZE];
		if (cr->PINCOS)
		{
			src_inc = cr->PINC * 4U;
		}
		else
		{
			src_inc = cr->PINC * src_size;
		}
	}
	else if (dir == DIR_M2P)
	{
		src = &s->regs.raw[chan_base+CH_OFF_SxM0AR];
		src_size = dma_xfer_size_b[cr->MSIZE];
		src_inc = cr->MINC * src_size;
		dest = &s->regs.raw[chan_base+CH_OFF_SxPAR];
		dest_size = dma_xfer_size_b[cr->PSIZE];
		if (cr->PINCOS)
		{
			dest_inc = cr->PINC * 4U;
		}
		else
		{
			dest_inc = cr->PINC * dest_size;
		}
	}
	else
	{
		printf("FIXME: M2M transfer!");
		abort();
	}
	// if (*src == 0x40004404)
	// {
	// 	printf("DMA Transfer: 0x%" PRIx32 "->0x%" PRIx32 ", size %u ndtr %u\n",*src, *dest, xfersize, *ndtr);
	// }
	uint8_t buff[4] = {0,0,0,0};
	dma_memory_read(
		&s->cpu_as,
		*src,
		buff,
		src_size,
		MEMTXATTRS_UNSPECIFIED
	);
	dma_memory_write(
		&s->cpu_as,
		*dest,
		buff,
		dest_size,
		MEMTXATTRS_UNSPECIFIED
	);

	(*ndtr)--; // NDTR is in transfers, not bytes.
	*dest += dest_inc;
	*src +=  src_inc;

	if (*ndtr == (s->original_ndtrs[channel]>>1U) )
	{
		stm32_f2xx_f4xx_set_int_flag(s, channel, INT_HTIF);
	}
	else if (*ndtr == 0 )
	{
		stm32_f2xx_f4xx_set_int_flag(s, channel, INT_TCIF);
		if (cr->CIRC)
		{
			*ndtr = s->original_ndtrs[channel];
			*dest -= (*ndtr*dest_inc);
			*src -= (*ndtr*src_inc);
		}
		else
		{
			cr->EN = false; // Transfer done, disable channel.
		}
	}
}

static void stm32_f2xx_f4xx_dma_dmar(void *opaque, int n, int level)
{
	//Idea: level contains the source address of the peripheral,
	// so that we don't have to look up a DR by peripheral ID like
	// in the STM32F4 implementation.
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);

	REGDEF_NAME(dma,sxcr) *cr;
	int idx = 0;
	for (int i=RI_CHAN_BASE; i<RI_CHAN_END; i+= CH_OFF_END, idx++)
	{
		cr = (REGDEF_NAME(dma,sxcr)*)&s->regs.raw[i+CH_OFF_SxCR];
		// Is this our peripheral?
		if (s->regs.raw[i+CH_OFF_SxPAR] != level)
		{
			continue;
		}
		if (cr->DIR != n) // Does the direction of the request match?
		{
			continue;
		}
		if (cr->EN != true || s->regs.raw[i+CH_OFF_SxNDTR] == 0)
		{
			continue;
		}
		s->dma_pending[idx]++;
		timer_mod_ns(s->dma_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
	}
}

static void stm32_f2xx_f4xx_dma_timer(void *opaque)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);
	for (int i=STM32_F2xx_DMA_MAX_CHAN - 1U; i>=0; i--)
	{
		if (s->dma_pending[i])
		{
			stm32_f2xx_f4xx_dma_do_xfer(s,RI_CHAN_BASE+(i*STM32_F2xx_DMA_CHAN_REGS));
			s->dma_pending[i]--;
		}
	}
}

static uint64_t
stm32_f2xx_f4xx_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= RI_MAX) { // LCOV_EXCL_START
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read stm32_dma register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    } // LCOV_EXCL_STOP

	// NOTE - IFCR is read-only, but we never change its contents in the write so it will always come back 0
    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b110);
	return value;
}

static void
stm32_f2xx_f4xx_dma_chan_write(STM32F2XX_STRUCT_NAME(Dma) *s, hwaddr addr, uint64_t data, unsigned int size)
{
	uint8_t chan = (addr - RI_CHAN_BASE)/STM32_F2xx_DMA_CHAN_REGS;
	uint8_t offset = (addr - RI_CHAN_BASE)%STM32_F2xx_DMA_CHAN_REGS;
	// printf("Wrote DMA channel %u/off %u with value %0"PRIx64"\n", chan, offset, data);
	switch (offset)
	{
		case CH_OFF_SxCR:
		{
			REGDEF_NAME(dma,sxcr) old = {.raw = s->regs.raw[addr]};
			REGDEF_NAME(dma,sxcr) new = {.raw = data};
			if (!old.EN && new.EN)
			{
				// printf("Enabled DMA channel %u\n", chan);
				// Channel was enabled... if P2M we wait for P to signal DMAR anyway.
			}
			else if (!new.EN && old.EN)
			{
				// DMA was disabled. Reset CMAR.
				s->regs.raw[addr+CH_OFF_SxM0AR] = s->original_cmars[chan];
				s->regs.raw[addr+CH_OFF_SxPAR] = s->original_cpars[chan];
			}
			s->regs.raw[addr] = new.raw;
		}
		break;
		case CH_OFF_SxNDTR:
			s->original_ndtrs[chan] = data & UINT16_MAX;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_SxPAR:
			s->original_cpars[chan] = data;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_SxM0AR:
			s->original_cmars[chan] = data;
			s->regs.raw[addr] = data;
			break;
		case CH_OFF_SxM1AR ... CH_OFF_SxFCR:
			s->regs.raw[addr] = data;
			break;
		default: //LCOV_EXCL_LINE
			qemu_log_mask(LOG_GUEST_ERROR, "ERR: DMA write to reserved register!\n"); // LCOV_EXCL_LINE
	}
}

static void
stm32_f2xx_f4xx_dma_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);

    int offset = addr & 0x3;

    addr >>= 2;

    if (addr >= RI_MAX) { // LCOV_EXCL_START
        qemu_log_mask(LOG_GUEST_ERROR, __FILE__ "invalid write stm32_iwdg register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    } // LCOV_EXCL_STOP

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b110);

    switch(addr) {
    case RI_LISR:
	case RI_HISR:
		qemu_log_mask(LOG_GUEST_ERROR, "ERR: Attempted to write read-only STM32 DMA ISR register.");
        break;
	case RI_LIFCR:
	case RI_HIFCR:
		s->regs.raw[addr - 2U] &= ~(data);
		stm32_f2xx_f4xx_update_all_irqs(s);
		break;
	case RI_CHAN_BASE ... RI_CHAN_END:
		stm32_f2xx_f4xx_dma_chan_write(s, addr, data, size);
        break;
    default: // LCOV_EXCL_START
        qemu_log_mask(LOG_UNIMP, __FILE__ " unimplemented write 0x%x+%u size %u val 0x%x\n",
        (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }  // LCOV_EXCL_STOP
}

static const MemoryRegionOps stm32_common_dma_ops = {
    .read = stm32_f2xx_f4xx_dma_read,
    .write = stm32_f2xx_f4xx_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};


static void stm32_f2xx_f4xx_dma_reset(DeviceState *dev)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(dev);
    memset(&s->regs, 0, sizeof(s->regs));
}

static void stm32_f2xx_f4xx_dma_realize(DeviceState *dev, Error **errp)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(dev);
	if (s->cpu_mr == NULL)
	{
		printf("No CPU memory region specified for %s - using global system memory.\n", _PERIPHNAMES[s->parent.periph]); // LCOV_EXCL_LINE
		s->cpu_mr = get_system_memory(); // LCOV_EXCL_LINE
	}
	gchar* name = g_strdup_printf("STM32COM_DMA_%d", s->parent.periph - STM32_P_DMA_BEGIN);
	address_space_init(&s->cpu_as, s->cpu_mr, name);
	g_free(name);
}

static void
stm32_f2xx_f4xx_dma_init(Object *obj)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(obj);


    assert(sizeof(s->regs.defs)==sizeof(s->regs.raw)); // Make sure packing is correct.
    // CHECK_REG_u32(s->regs.defs.ISR);
    CHECK_REG_u32(s->regs.defs.LIFCR);
    CHECK_REG_u32(s->regs.defs.HIFCR);
    CHECK_REG_u32(s->regs.defs.LISR);
    CHECK_REG_u32(s->regs.defs.HISR);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, sxcr),  SxCR);
	// CHECK_TYPEDEF_u32(REGDEF_NAME(dma, sxfcr), SxFCR);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, interrupt), defs.IFCR);
    // CHECK_TYPEDEF_u32(REGDEF_NAME(dma, ccr), defs.CCR);


    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_dma_ops, s, 1*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in_named(DEVICE(obj),stm32_f2xx_f4xx_dma_dmar,"dmar-in",2);
	qdev_init_gpio_out_named(DEVICE(obj), s->irq, SYSBUS_DEVICE_GPIO_IRQ, STM32_F2xx_DMA_MAX_CHAN);

	s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_f2xx_f4xx_dma_timer, obj);
}

static const VMStateDescription vmstate_stm32_f2xx_f4xx_dma = {
    .name = TYPE_STM32F2xx_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32F2XX_STRUCT_NAME(Dma), RI_MAX),
        VMSTATE_UINT32_ARRAY(original_ndtrs,STM32F2XX_STRUCT_NAME(Dma), STM32_F2xx_DMA_MAX_CHAN),
        VMSTATE_UINT32_ARRAY(original_cmars,STM32F2XX_STRUCT_NAME(Dma), STM32_F2xx_DMA_MAX_CHAN),
        VMSTATE_UINT32_ARRAY(original_cpars,STM32F2XX_STRUCT_NAME(Dma), STM32_F2xx_DMA_MAX_CHAN),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32_f2xx_f4xx_dma_properties[] = {
    DEFINE_PROP_LINK("system-memory", STM32F2XX_STRUCT_NAME(Dma), cpu_mr, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_f2xx_f4xx_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_f2xx_f4xx_dma;
    dc->reset = stm32_f2xx_f4xx_dma_reset;
	dc->realize = stm32_f2xx_f4xx_dma_realize;
	device_class_set_props(dc, stm32_f2xx_f4xx_dma_properties);
}

static const TypeInfo
stm32_f2xx_dma_info = {
    .name          = TYPE_STM32F2xx_DMA,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F2XX_STRUCT_NAME(Dma)),
    .instance_init = stm32_f2xx_f4xx_dma_init,
    .class_init    = stm32_f2xx_f4xx_dma_class_init,
};

static const TypeInfo
stm32_f4xx_dma_info = {
    .name          = TYPE_STM32F4xx_DMA,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F2XX_STRUCT_NAME(Dma)),
    .instance_init = stm32_f2xx_f4xx_dma_init,
    .class_init    = stm32_f2xx_f4xx_dma_class_init,
};

static void
stm32_f2xx_f4xx_dma_register_types(void)
{
    type_register_static(&stm32_f2xx_dma_info);
    type_register_static(&stm32_f4xx_dma_info);
}

type_init(stm32_f2xx_f4xx_dma_register_types)
