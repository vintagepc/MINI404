/*
    stm32_dma.h - DMA for STM32F2/4.
	So far the DMA layout is common to the following chips:
	- F4xx series,
	- F2xx series (? - it's modelled off the Pebble QEMU version which supports this.),

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

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

#include "stm32_f2xx_f4xx_dma.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "sysemu/dma.h"
#include "qemu/log.h"

enum reg_index {
	RI_LISR,
	RI_HISR,
	RI_LIFCR,
	RI_HIFCR,
	RI_CHAN_BASE,
	RI_CHAN_END = RI_CHAN_BASE + (STM32_F2xx_DMA_CHAN_REGS * STM32_F2xx_DMA_MAX_CHAN),
	RI_MAX = RI_CHAN_END,
};

enum channel_offset
{
	CH_OFF_SxCR,
	CH_OFF_SxNDTR,
	CH_OFF_SxPAR,
	CH_OFF_SxM0AR,
	CH_OFF_SxM1AR,
	CH_OFF_SxFCR,
	CH_OFF_END,
};

enum dma_dir
{
	DIR_P2M,
	DIR_M2P,
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

QEMU_BUILD_BUG_MSG(R_DMA_MAX != RI_MAX, "register definitions do not agree!");
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

static void stm32_f2xx_f4xx_dma_do_p2m(STM32F2XX_STRUCT_NAME(Dma) *s, hwaddr chan_base)
{
	uint8_t channel = (chan_base - RI_CHAN_BASE) / STM32_F2xx_DMA_CHAN_REGS;
	REGDEF_NAME(dma,sxcr)* cr = (REGDEF_NAME(dma,sxcr)*)&s->regs.raw[chan_base+CH_OFF_SxCR];
	uint8_t psize = dma_xfer_size_b[cr->PSIZE];
	uint8_t msize = dma_xfer_size_b[cr->MSIZE];
	uint8_t xfersize = MIN(psize, msize);
	uint32_t *dest = &s->regs.raw[chan_base+CH_OFF_SxM0AR];
	uint32_t *src = &s->regs.raw[chan_base+CH_OFF_SxPAR];
	uint32_t *ndtr = &s->regs.raw[chan_base+CH_OFF_SxNDTR];
	// if (*src == 0x40004404)
	// {
	// 	printf("DMA Transfer: 0x%" PRIx32 "->0x%" PRIx32 ", size %u ndtr %u\n",*src, *dest, xfersize, *ndtr);
	// }
	uint8_t buff[4];
	dma_memory_read(
		&address_space_memory,
		*src,
		buff,
		xfersize
	);
	dma_memory_write(
		&address_space_memory,
		*dest,
		buff,
		xfersize
	);
	if (cr->MINC)
	{
		*dest += xfersize;
	}
	if (cr->PINC)
	{
		*src += xfersize;
	}
	(*ndtr)--; // NDTR is in transfers, not bytes.

	if (*ndtr == 0 )
	{
		// if (*src == 0x40004404)
		// {
		// 	printf("TCIF\n");
		// }
		stm32_f2xx_f4xx_set_int_flag(s, channel, INT_TCIF);
	}

	if (*ndtr == (s->original_ndtrs[channel]>>1U) )
	{
		stm32_f2xx_f4xx_set_int_flag(s, channel, INT_HTIF);
	}
	if (cr->CIRC && *ndtr == 0)
	{
		*ndtr = s->original_ndtrs[channel];
		if (cr->MINC)
		{
			*dest -= (*ndtr*msize);
		}
	}
}

static void stm32_f2xx_f4xx_dma_do_m2p(STM32F2XX_STRUCT_NAME(Dma) *s, hwaddr chan_base)
{
	uint8_t channel = (chan_base - RI_CHAN_BASE) / STM32_F2xx_DMA_CHAN_REGS;
	REGDEF_NAME(dma,sxcr)* cr = (REGDEF_NAME(dma,sxcr)*)&s->regs.raw[chan_base+CH_OFF_SxCR];
	uint8_t psize = dma_xfer_size_b[cr->PSIZE];
	uint8_t msize = dma_xfer_size_b[cr->MSIZE];
	uint8_t xfersize = MIN(psize, msize);
	uint32_t *dest = &s->regs.raw[chan_base+CH_OFF_SxPAR];
	uint32_t *src = &s->regs.raw[chan_base+CH_OFF_SxM0AR];
	uint32_t *ndtr = &s->regs.raw[chan_base+CH_OFF_SxNDTR];
	// printf("DMA Transfer: 0x%" PRIx32 "->0x%" PRIx32 ", size %u ndtr %u\n",*src, *dest, xfersize, *ndtr);
	uint8_t buff[4];
	while (*ndtr)
	{
		dma_memory_read(
			&address_space_memory,
			*src,
			buff,
			xfersize
		);
		dma_memory_write(
			&address_space_memory,
			*dest,
			buff,
			xfersize
		);
		if (cr->MINC)
		{
			*src += xfersize;
		}
		if (cr->PINC)
		{
			*dest += xfersize;
		}
		(*ndtr)--; // NDTR is in transfers, not bytes.
	}

	if (*ndtr == 0 )
	{
		stm32_f2xx_f4xx_set_int_flag(s, channel, INT_TCIF);
	}
	if (cr->CIRC && *ndtr == 0)
	{
		printf("FIXME: CIRC MODE M2P\n");
	}
}

static void stm32_f2xx_f4xx_dma_dmar(void *opaque, int n, int level)
{
	//Idea: level contains the source address of the peripheral,
	// so that we don't have to look up a DR by peripheral ID like
	// in the STM32F4 implementation.
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);

	REGDEF_NAME(dma,sxcr) *cr;
	for (int i=RI_CHAN_BASE; i<RI_CHAN_END; i+= CH_OFF_END)
	{
		cr = (REGDEF_NAME(dma,sxcr)*)&s->regs.raw[i+CH_OFF_SxCR];
		// Is this our peripheral?
		if (s->regs.raw[i+CH_OFF_SxPAR] != level)
		{
			continue;
		}
		if (cr->EN != true || s->regs.raw[i+CH_OFF_SxNDTR] == 0)
			continue;

		if (cr->DIR==DIR_P2M)
		{
			stm32_f2xx_f4xx_dma_do_p2m(s, i);
		}
		else
		{
			printf("FIXME: M2P/M2M DMA transfer!\n");
		}

	}
}

static uint64_t
stm32_f2xx_f4xx_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read stm32_dma register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }

	// NOTE - IFCR is read-only, but we never change its contents in the write so it will always come back 0
    uint32_t value = s->regs.raw[addr];

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);
    return r;
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
				if (new.DIR == DIR_M2P)
				{
					// If M2P, start the transfer now and don't bother setting EN

					stm32_f2xx_f4xx_dma_do_m2p(s, addr - offset);
					new.EN = 0;
				}
			}
			s->regs.raw[addr] = new.raw;
		}
		break;
		case CH_OFF_SxNDTR:
			s->original_ndtrs[chan] = data & UINT16_MAX;
		/* FALLTHRU */
		case CH_OFF_SxPAR ... CH_OFF_SxFCR:
			s->regs.raw[addr] = data;
			break;
		default:
			qemu_log_mask(LOG_GUEST_ERROR, "ERR: DMA write to reserved register!\n");
	}
}

static void
stm32_f2xx_f4xx_dma_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32F2XX_STRUCT_NAME(Dma) *s = STM32F4xx_DMA(opaque);

    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= RI_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, __FILE__ "invalid write stm32_iwdg register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 2:
        data = (s->regs.raw[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }

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
    default:
        qemu_log_mask(LOG_UNIMP, __FILE__ " unimplemented write 0x%x+%u size %u val 0x%x\n",
        (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
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


    memory_region_init_io(&s->iomem, obj, &stm32_common_dma_ops, s, "dma", 1*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in_named(DEVICE(obj),stm32_f2xx_f4xx_dma_dmar,"dmar-in",1);
	qdev_init_gpio_out_named(DEVICE(obj), s->irq, SYSBUS_DEVICE_GPIO_IRQ, STM32_F2xx_DMA_MAX_CHAN);

}

static const VMStateDescription vmstate_stm32_f2xx_f4xx_dma = {
    .name = TYPE_STM32F2xx_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32F2XX_STRUCT_NAME(Dma), R_DMA_MAX),
        VMSTATE_UINT32_ARRAY(original_ndtrs,STM32F2XX_STRUCT_NAME(Dma), STM32_F2xx_DMA_MAX_CHAN),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32_f2xx_f4xx_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_f2xx_f4xx_dma;
    dc->reset = stm32_f2xx_f4xx_dma_reset;
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

