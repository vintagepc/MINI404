/*
 * STM32F4XX SPI
 * Original Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified/rewritten/bugfixed for Mini404 by VintagePC 2021-2022 (http://github.com/vintagepc/)
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
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/ssi.h"
#include "hw/irq.h"
#include "stm32_shared.h"
#include "stm32_common.h"
#include "migration/vmstate.h"
#include "stm32_rcc_if.h"

enum reg_index {
	RI_CR1,
	RI_CR2,
	RI_SR,
	RI_DR,
	RI_CRCPR,
	RI_RXCRCR,
	RI_TXCRCR,
	RI_I2SCFGR,
	RI_I2SPR,
	RI_END,
};


OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Spi), COM_CLASS_NAME(Spi), STM32COM_SPI);

REGDEF_BLOCK_BEGIN()
	REG_B32(CPHA);
	REG_B32(CPOL);
	REG_B32(MSTR);
	REG_K32(BR, 3);
	REG_B32(SPE);
	REG_B32(LSBFIRST);
	REG_B32(SSI);
	REG_B32(SSM);
	REG_B32(RXONLY);
	REG_B32(DFF);
	REG_B32(CRCNEXT);
	REG_B32(CRCEN);
	REG_B32(BIDIOE);
	REG_B32(BIDIMODE);
	REG_R(16);
REGDEF_BLOCK_END(spi, cr1);

REGDEF_BLOCK_BEGIN()
	REG_B32(RXDMAEN);
	REG_B32(TXDMAEN);
	REG_B32(SSOE);
	REG_B32(NSSP);
	REG_B32(FRF);
	REG_B32(ERRIE);
	REG_B32(RXNEIE);
	REG_B32(TXEIE);
	REG_K32(DS,4);
	REG_B32(FRXTH);
	REG_B32(LDMA_RX);
	REG_B32(LDMA_TX);
	REG_R(17);
REGDEF_BLOCK_END(spi, cr2);

REGDEF_BLOCK_BEGIN()
	REG_B32(RXNE);
	REG_B32(TXE);
	REG_B32(CHSIDE);
	REG_B32(UDR);
	REG_B32(CRCERR);
	REG_B32(MODF);
	REG_B32(OVR);
	REG_B32(BSY);
	REG_B32(FRE);
	REG_K32(FRLVL, 2);
	REG_K32(FTLVL, 2);
	REG_R(19);
REGDEF_BLOCK_END(spi, sr);

REGDEF_BLOCK_BEGIN()
	REG_B32(CHLEN);
	REG_K32(DATLEN,2);
	REG_B32(CKPOL);
	REG_K32(I2STD, 2);
	REG_RB();
	REG_B32(PCMSYNC);
	REG_K32(I2SCFG, 2);
	REG_B32(I2SE);
	REG_B32(I2SMOD);
	REG_B32(ASTREN);
	REG_R(19);
REGDEF_BLOCK_END(spi, i2scfgr);

REGDEF_BLOCK_BEGIN()
	REG_K32(I2SDIV, 8);
	REG_B32(ODD);
	REG_B32(MCKOE);
	REG_R(22);
REGDEF_BLOCK_END(spi, i2spr);

typedef struct COM_STRUCT_NAME(Spi) {
    /* <private> */
    STM32Peripheral parent;

    /* <public> */
    MemoryRegion mmio;

	union {
		struct {
			REGDEF_NAME(spi, cr1) CR1;
			REGDEF_NAME(spi, cr2) CR2;
			REGDEF_NAME(spi, sr) SR;
			REG_S32(DR, 16);
			REG_S32(CRCPR, 16);
			REG_S32(RXCRCR, 16);
			REG_S32(TXCRCR, 16);
			REGDEF_NAME(spi, i2scfgr) I2SCFGR;
			REGDEF_NAME(spi, i2spr) I2SPR;
		} defs;
		uint32_t raw[RI_END];
	} regs;

    qemu_irq irq;
    SSIBus *ssi;

	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Spi);

typedef struct COM_CLASS_NAME(Spi) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Spi);

static const stm32_reginfo_t stm32f030_spi_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = UINT16_MAX, .unimp_mask = 0x800},
	[RI_CR2] = {.mask = 0x7FFF, .reset_val = 0x700},
	[RI_SR] = {.mask = 0x1FF3, .reset_val = 0x00000002},
	[RI_DR] = {.mask = UINT16_MAX},
	[RI_CRCPR] = {.mask = UINT16_MAX, .reset_val = 0x00000007},
	[RI_RXCRCR] = {.mask = UINT16_MAX},
	[RI_TXCRCR] = {.mask = UINT16_MAX},
	[RI_I2SCFGR] = {.is_reserved = true},
	[RI_I2SPR] = {.is_reserved = true},
};

static const stm32_reginfo_t stm32g070_spi_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = UINT16_MAX, .unimp_mask = 0x800},
	[RI_CR2] = {.mask = 0x7FFF, .reset_val = 0x700},
	[RI_SR] = {.mask = 0x1FF3, .reset_val = 0x00000002},
	[RI_DR] = {.mask = UINT16_MAX},
	[RI_CRCPR] = {.mask = UINT16_MAX, .reset_val = 0x00000007},
	[RI_RXCRCR] = {.mask = UINT16_MAX},
	[RI_TXCRCR] = {.mask = UINT16_MAX},
	[RI_I2SCFGR] = {.mask = 0x1FBF, .unimp_mask = 0x1FBF},
	[RI_I2SPR] = {.mask = 0x3FF, .unimp_mask = 0x3FF, .reset_val = 0x2},
};


static const stm32_reginfo_t stm32f2xx_spi_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = UINT16_MAX, .unimp_mask = 0x800},
	[RI_CR2] = {.mask = 0xF7},
	[RI_SR] = {.mask = 0x1FF, .reset_val = 0x0000000A},
	[RI_DR] = {.mask = UINT16_MAX},
	[RI_CRCPR] = {.mask = UINT16_MAX, .reset_val = 0x00000007},
	[RI_RXCRCR] = {.mask = UINT16_MAX},
	[RI_TXCRCR] = {.mask = UINT16_MAX},
	[RI_I2SCFGR] = {.mask = 0xFFF, .unimp_mask = 0xFFF},
	[RI_I2SPR] = {.mask = 0x3FF, .unimp_mask = 0x3FF, .reset_val = 0x2},
};

static const stm32_reginfo_t stm32_common_spi_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = UINT16_MAX, .unimp_mask = 0x800},
	[RI_CR2] = {.mask = 0xF7},
	[RI_SR] = {.mask = 0x1FF, .reset_val = 0x00000002},
	[RI_DR] = {.mask = UINT16_MAX},
	[RI_CRCPR] = {.mask = UINT16_MAX, .reset_val = 0x00000007},
	[RI_RXCRCR] = {.mask = UINT16_MAX},
	[RI_TXCRCR] = {.mask = UINT16_MAX},
	[RI_I2SCFGR] = {.mask = 0xFFF, .unimp_mask = 0xFFF},
	[RI_I2SPR] = {.mask = 0x3FF, .unimp_mask = 0x3FF, .reset_val = 0x2},
};

static const stm32_periph_variant_t stm32_spi_variants[4] = {
	{TYPE_STM32F030_SPI, stm32f030_spi_reginfo},
	{TYPE_STM32G070_SPI, stm32g070_spi_reginfo},
	{TYPE_STM32F2xx_SPI, stm32f2xx_spi_reginfo},
	{TYPE_STM32F4xx_SPI, stm32_common_spi_reginfo}
};

static void stm32_common_spi_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(Spi) *s = STM32COM_SPI(dev);
	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
}

static uint64_t stm32_common_spi_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    COM_STRUCT_NAME(Spi) *s = STM32COM_SPI(opaque);

	if ((addr & 0x3) !=0)
	{
		qemu_log_mask(LOG_GUEST_ERROR, __FILE__ ": Unaligned SPI register read\n");
	}

    addr >>= 2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32 SPI");

    switch (addr) {
    	case RI_DR:
        	s->regs.defs.SR.RXNE = 0;
			/* FALLTHRU */
		default:
			return s->regs.raw[addr];

    }
}

static uint8_t bitswap(uint8_t val)
{
    return ((val * 0x0802LU & 0x22110LU) | (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}


static void stm32_common_spi_write(void *opaque, hwaddr addr,
                                uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Spi) *s = STM32COM_SPI(opaque);

    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32COM SPI");

	CHECK_UNIMP_RESVD(data, s->reginfo, addr);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 7);

    switch (addr) {
        case RI_CR1:
		{
            REGDEF_NAME(spi, cr1) new = {.raw = data};
            if (s->regs.defs.CR1.SPE && new.DFF){
                qemu_log_mask(LOG_GUEST_ERROR, "cannot change DFF with SPE set\n");
			}
            s->regs.raw[RI_CR1] = data;
		}
		break;
		case RI_CR2:
		{
            REGDEF_NAME(spi, cr2) new = {.raw = data};
			if (new.TXDMAEN && !s->regs.defs.CR2.TXDMAEN)
			{
				qemu_set_irq(s->parent.dmar[DMAR_M2P], s->mmio.addr + (4U * RI_DR));
			}
            s->regs.raw[RI_CR2] = data;
		}
		break;
        case RI_DR:
		{
            REGDEF_NAME(spi, sr)* sr = &s->regs.defs.SR;
			sr->TXE = false;
            if (sr->RXNE) {
                sr->OVR = true;
            }
            if (s->regs.defs.CR1.LSBFIRST) {
                s->regs.defs.DR = bitswap(ssi_transfer(s->ssi, bitswap(data)));
            } else {
                s->regs.defs.DR = ssi_transfer(s->ssi, data);
            }
            sr->RXNE = true;
			if (s->regs.defs.CR2.RXDMAEN)
			{
				qemu_set_irq(s->parent.dmar[DMAR_P2M], s->mmio.addr + (4U * RI_DR));
			}
            sr->TXE = true;
			if (s->regs.defs.CR2.TXDMAEN)
			{
				qemu_set_irq(s->parent.dmar[DMAR_M2P], s->mmio.addr + (4U * RI_DR));
			}
		}
		break;
    default:
		s->regs.raw[addr] = data;
    }
}

static const MemoryRegionOps stm32_common_spi_ops = {
    .read = stm32_common_spi_read,
    .write = stm32_common_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32_common_spi = {
    .name = TYPE_STM32COM_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,COM_STRUCT_NAME(Spi), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_common_spi_init(Object *obj)
{
	COM_STRUCT_NAME(Spi) *s = STM32COM_SPI(obj);
    CHECK_ALIGN(sizeof(s->regs),sizeof(s->regs.raw), "SPI"); // Make sure packing is correct.
	CHECK_REGDEF_u32(REGDEF_NAME(spi, cr1),CR1);
	CHECK_REGDEF_u32(REGDEF_NAME(spi, cr2),CR1);
	CHECK_REGDEF_u32(REGDEF_NAME(spi, sr),SR);
	CHECK_REGDEF_u32(REGDEF_NAME(spi, i2scfgr),I2SCFGR);
	CHECK_REGDEF_u32(REGDEF_NAME(spi, i2spr),I2SPR);
	CHECK_UNION(COM_STRUCT_NAME(Spi), regs.raw[RI_CR1], regs.defs.CR1);
	CHECK_UNION(COM_STRUCT_NAME(Spi), regs.raw[RI_CR2], regs.defs.CR2);
	CHECK_UNION(COM_STRUCT_NAME(Spi), regs.raw[RI_SR], regs.defs.SR);
	CHECK_UNION(COM_STRUCT_NAME(Spi), regs.raw[RI_I2SCFGR], regs.defs.I2SCFGR);
	CHECK_UNION(COM_STRUCT_NAME(Spi), regs.raw[RI_I2SPR], regs.defs.I2SPR);

    DeviceState *dev = DEVICE(obj);

    STM32_MR_IO_INIT(&s->mmio, obj, &stm32_common_spi_ops, s, 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->ssi = ssi_create_bus(dev, "ssi");

	COM_CLASS_NAME(Spi) *k = STM32COM_SPI_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
}

static void stm32_common_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32_common_spi_reset;
    dc->vmsd = &vmstate_stm32_common_spi;

	COM_CLASS_NAME(Spi) *k = STM32COM_SPI_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo stm32_common_spi_info = {
    .name          = TYPE_STM32COM_SPI,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Spi)),
	.class_size = sizeof(COM_CLASS_NAME(Spi)),
	.abstract = true,
};

static void stm32_common_spi_register_types(void)
{
    type_register_static(&stm32_common_spi_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_spi_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_spi_variants[i].variant_name,
            .parent     = TYPE_STM32COM_SPI,
			.instance_init = stm32_common_spi_init,
    		.class_init    = stm32_common_spi_class_init,
            .class_data = (void *)stm32_spi_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_spi_register_types)
