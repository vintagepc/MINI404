/*-
 * QEMU crc emulation
 * Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020-3 by VintagePC <http://github.com/vintagepc>
 * Further refined to be shared among the entire STM32 family in 2021
 *
 * This layout is known to be shared amongst the following SOCs:
 * - STM32F030x
 * - STM32G070
 * - STM32F2xx (given it originally appeared in the Pebble QEMU fork and was adapted)
 * - STM32F4xx
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
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "stm32_types.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "stm32_rcc_if.h"
#include "stm32_crc_regdata.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Crc), COM_CLASS_NAME(Crc), STM32COM_CRC);

REGDEF_BLOCK_BEGIN()
	REG_B32(RESET);
	REG_R(4);
	REG_K32(REV_IN,2);
	REG_B32(REV_OUT);
	REG_R(24);
REGDEF_BLOCK_END(crc, cr);

typedef struct COM_STRUCT_NAME(Crc) {
    STM32Peripheral parent;
    MemoryRegion iomem;

	union {
		struct {
			uint32_t DR;
			uint32_t IDR;
			REGDEF_NAME(crc,cr) CR;
			uint32_t _reserved;
			uint32_t INIT;
			uint32_t POL;
		} defs;
		uint32_t raw[RI_END];
	} regs;

	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Crc);


/*****************************************************************/
/*                                                               */
/* CRC LOOKUP TABLE                                              */
/* ================                                              */
/* The following CRC lookup table was generated automagically    */
/* by the Rocksoft^tm Model CRC Algorithm Table Generation       */
/* Program V1.0 using the following model parameters:            */
/*                                                               */
/*    Width   : 4 bytes.                                         */
/*    Poly    : 0x04C11DB7L                                      */
/*    Reverse : FALSE.                                           */
/*                                                               */
/* For more information on the Rocksoft^tm Model CRC Algorithm,  */
/* see the document titled "A Painless Guide to CRC Error        */
/* Detection Algorithms" by Ross Williams                        */
/* (ross@guest.adelaide.edu.au.). This document is likely to be  */
/* in the FTP archive "ftp.adelaide.edu.au/pub/rocksoft".        */
/*                                                               */
/*****************************************************************/

static unsigned long crctable[256] =
{
 0x00000000L, 0x04C11DB7L, 0x09823B6EL, 0x0D4326D9L, 0x130476DCL, 0x17C56B6BL,
 0x1A864DB2L, 0x1E475005L, 0x2608EDB8L, 0x22C9F00FL, 0x2F8AD6D6L, 0x2B4BCB61L,
 0x350C9B64L, 0x31CD86D3L, 0x3C8EA00AL, 0x384FBDBDL, 0x4C11DB70L, 0x48D0C6C7L,
 0x4593E01EL, 0x4152FDA9L, 0x5F15ADACL, 0x5BD4B01BL, 0x569796C2L, 0x52568B75L,
 0x6A1936C8L, 0x6ED82B7FL, 0x639B0DA6L, 0x675A1011L, 0x791D4014L, 0x7DDC5DA3L,
 0x709F7B7AL, 0x745E66CDL, 0x9823B6E0L, 0x9CE2AB57L, 0x91A18D8EL, 0x95609039L,
 0x8B27C03CL, 0x8FE6DD8BL, 0x82A5FB52L, 0x8664E6E5L, 0xBE2B5B58L, 0xBAEA46EFL,
 0xB7A96036L, 0xB3687D81L, 0xAD2F2D84L, 0xA9EE3033L, 0xA4AD16EAL, 0xA06C0B5DL,
 0xD4326D90L, 0xD0F37027L, 0xDDB056FEL, 0xD9714B49L, 0xC7361B4CL, 0xC3F706FBL,
 0xCEB42022L, 0xCA753D95L, 0xF23A8028L, 0xF6FB9D9FL, 0xFBB8BB46L, 0xFF79A6F1L,
 0xE13EF6F4L, 0xE5FFEB43L, 0xE8BCCD9AL, 0xEC7DD02DL, 0x34867077L, 0x30476DC0L,
 0x3D044B19L, 0x39C556AEL, 0x278206ABL, 0x23431B1CL, 0x2E003DC5L, 0x2AC12072L,
 0x128E9DCFL, 0x164F8078L, 0x1B0CA6A1L, 0x1FCDBB16L, 0x018AEB13L, 0x054BF6A4L,
 0x0808D07DL, 0x0CC9CDCAL, 0x7897AB07L, 0x7C56B6B0L, 0x71159069L, 0x75D48DDEL,
 0x6B93DDDBL, 0x6F52C06CL, 0x6211E6B5L, 0x66D0FB02L, 0x5E9F46BFL, 0x5A5E5B08L,
 0x571D7DD1L, 0x53DC6066L, 0x4D9B3063L, 0x495A2DD4L, 0x44190B0DL, 0x40D816BAL,
 0xACA5C697L, 0xA864DB20L, 0xA527FDF9L, 0xA1E6E04EL, 0xBFA1B04BL, 0xBB60ADFCL,
 0xB6238B25L, 0xB2E29692L, 0x8AAD2B2FL, 0x8E6C3698L, 0x832F1041L, 0x87EE0DF6L,
 0x99A95DF3L, 0x9D684044L, 0x902B669DL, 0x94EA7B2AL, 0xE0B41DE7L, 0xE4750050L,
 0xE9362689L, 0xEDF73B3EL, 0xF3B06B3BL, 0xF771768CL, 0xFA325055L, 0xFEF34DE2L,
 0xC6BCF05FL, 0xC27DEDE8L, 0xCF3ECB31L, 0xCBFFD686L, 0xD5B88683L, 0xD1799B34L,
 0xDC3ABDEDL, 0xD8FBA05AL, 0x690CE0EEL, 0x6DCDFD59L, 0x608EDB80L, 0x644FC637L,
 0x7A089632L, 0x7EC98B85L, 0x738AAD5CL, 0x774BB0EBL, 0x4F040D56L, 0x4BC510E1L,
 0x46863638L, 0x42472B8FL, 0x5C007B8AL, 0x58C1663DL, 0x558240E4L, 0x51435D53L,
 0x251D3B9EL, 0x21DC2629L, 0x2C9F00F0L, 0x285E1D47L, 0x36194D42L, 0x32D850F5L,
 0x3F9B762CL, 0x3B5A6B9BL, 0x0315D626L, 0x07D4CB91L, 0x0A97ED48L, 0x0E56F0FFL,
 0x1011A0FAL, 0x14D0BD4DL, 0x19939B94L, 0x1D528623L, 0xF12F560EL, 0xF5EE4BB9L,
 0xF8AD6D60L, 0xFC6C70D7L, 0xE22B20D2L, 0xE6EA3D65L, 0xEBA91BBCL, 0xEF68060BL,
 0xD727BBB6L, 0xD3E6A601L, 0xDEA580D8L, 0xDA649D6FL, 0xC423CD6AL, 0xC0E2D0DDL,
 0xCDA1F604L, 0xC960EBB3L, 0xBD3E8D7EL, 0xB9FF90C9L, 0xB4BCB610L, 0xB07DABA7L,
 0xAE3AFBA2L, 0xAAFBE615L, 0xA7B8C0CCL, 0xA379DD7BL, 0x9B3660C6L, 0x9FF77D71L,
 0x92B45BA8L, 0x9675461FL, 0x8832161AL, 0x8CF30BADL, 0x81B02D74L, 0x857130C3L,
 0x5D8A9099L, 0x594B8D2EL, 0x5408ABF7L, 0x50C9B640L, 0x4E8EE645L, 0x4A4FFBF2L,
 0x470CDD2BL, 0x43CDC09CL, 0x7B827D21L, 0x7F436096L, 0x7200464FL, 0x76C15BF8L,
 0x68860BFDL, 0x6C47164AL, 0x61043093L, 0x65C52D24L, 0x119B4BE9L, 0x155A565EL,
 0x18197087L, 0x1CD86D30L, 0x029F3D35L, 0x065E2082L, 0x0B1D065BL, 0x0FDC1BECL,
 0x3793A651L, 0x3352BBE6L, 0x3E119D3FL, 0x3AD08088L, 0x2497D08DL, 0x2056CD3AL,
 0x2D15EBE3L, 0x29D4F654L, 0xC5A92679L, 0xC1683BCEL, 0xCC2B1D17L, 0xC8EA00A0L,
 0xD6AD50A5L, 0xD26C4D12L, 0xDF2F6BCBL, 0xDBEE767CL, 0xE3A1CBC1L, 0xE760D676L,
 0xEA23F0AFL, 0xEEE2ED18L, 0xF0A5BD1DL, 0xF464A0AAL, 0xF9278673L, 0xFDE69BC4L,
 0x89B8FD09L, 0x8D79E0BEL, 0x803AC667L, 0x84FBDBD0L, 0x9ABC8BD5L, 0x9E7D9662L,
 0x933EB0BBL, 0x97FFAD0CL, 0xAFB010B1L, 0xAB710D06L, 0xA6322BDFL, 0xA2F33668L,
 0xBCB4666DL, 0xB8757BDAL, 0xB5365D03L, 0xB1F740B4L
};

typedef struct COM_CLASS_NAME(Crc) {
	SysBusDeviceClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Crc);




static const stm32_periph_variant_t stm32_crc_variants[4] = {
	{TYPE_STM32F030_CRC, stm32f030_crc_reginfo},
	{TYPE_STM32G070_CRC, stm32g070_crc_reginfo},
	{TYPE_STM32F2xx_CRC, stm32f2xx_crc_reginfo},
	{TYPE_STM32F4xx_CRC, stm32f4xx_crc_reginfo}
};

static inline uint32_t
update_crc(uint32_t crc, uint8_t byte)
{
    return crctable[((crc >> 24) ^ byte) & 0xff] ^ (crc << 8);
}

static uint64_t
stm32_common_crc_read(void *arg, hwaddr addr, unsigned int size)
{
    COM_STRUCT_NAME(Crc) *s = STM32COM_CRC(arg);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "CRC only allows 4-byte reads\n");
        return 0;
    }

    addr >>= 2;
	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32 CRC");

    switch(addr) {
    case RI_DR:
        if (stm32_rcc_if_check_periph_clk(&s->parent))
        {
            return s->regs.defs.DR;
        }
        else
        {
            return 0;
        }
    default:
        return s->regs.raw[addr];
    }
    return 0;
}


static void
stm32_common_crc_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    COM_STRUCT_NAME(Crc) *s = STM32COM_CRC(arg);

	uint8_t offset = addr & 0x03;
    addr >>= 2;
    /* XXX Check periph clock enable. */

	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32 CRC");

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b111);

	if (size != 4 && addr != RI_DR)
	{
        qemu_log_mask(LOG_GUEST_ERROR, "CRC only allows 4-byte writes for registers other than DR\n");
		return;
	}


    switch(addr) {
    case RI_DR:
        s->regs.defs.DR = update_crc(s->regs.defs.DR, (data >> 24) & 0xff);
        s->regs.defs.DR = update_crc(s->regs.defs.DR, (data >> 16) & 0xff);
        s->regs.defs.DR = update_crc(s->regs.defs.DR, (data >> 8) & 0xff);
        s->regs.defs.DR = update_crc(s->regs.defs.DR, data & 0xff);
        break;
    case RI_IDR:
		ENFORCE_RESERVED(data, s->reginfo, RI_IDR);
        s->regs.defs.IDR = data;
        break;
    case RI_CR:
		CHECK_UNIMP_RESVD(data, s->reginfo, RI_CR);
		s->regs.defs.CR.raw = data;
        if (s->regs.defs.CR.RESET) {
            s->regs.defs.DR = s->regs.defs.INIT;
			s->regs.defs.CR.RESET = 0;
        }
		break;
	case RI_INIT:
		s->regs.defs.INIT = data;
		s->regs.defs.DR = data;
        break;
    }
}

static const MemoryRegionOps stm32_common_crc_ops = {
    .read = stm32_common_crc_read,
    .write = stm32_common_crc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};


static void
stm32_common_crc_reset(DeviceState *ds)
{
    COM_STRUCT_NAME(Crc) *s = STM32COM_CRC(ds);

	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
}

static void
stm32_common_crc_init(Object *obj)
{
    COM_STRUCT_NAME(Crc) *s = STM32COM_CRC(obj);
	CHECK_ALIGN(sizeof(s->regs.raw), sizeof(s->regs.defs), "Union misaligned!");
	CHECK_ALIGN(sizeof(s->regs.defs), sizeof(uint32_t)*RI_END, "Union overall size");
	CHECK_REG_u32(s->regs.defs.CR);

	COM_CLASS_NAME(Crc) *k = STM32COM_CRC_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_crc_ops, s, 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}


static const VMStateDescription vmstate_stm32stm32_common_crc = {
    .name = TYPE_STM32COM_CRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Crc), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_crc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_common_crc_reset;
    dc->vmsd = &vmstate_stm32stm32_common_crc;

	COM_CLASS_NAME(Crc) *k = STM32COM_CRC_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");

}

static const TypeInfo
stm32_common_crc_info = {
    .name          = TYPE_STM32COM_CRC,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Crc)),
	.class_size	   = sizeof(COM_CLASS_NAME(Crc)),
	.abstract	   = true,
};

static void
stm32_common_crc_register_types(void)
{
    type_register_static(&stm32_common_crc_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_crc_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_crc_variants[i].variant_name,
            .parent     = TYPE_STM32COM_CRC,
            .class_init = stm32_common_crc_class_init,
            .class_data = (void *)stm32_crc_variants[i].variant_regs,
			.instance_init = stm32_common_crc_init,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_crc_register_types)
