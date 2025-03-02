/*
 * stm32_common ADC Common register
 *
 * Copyright (c) 2022 by VintagePC <http://github.com/vintagepc> for Mini404
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
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/typedefs.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "stm32_adcc.h"
#include "../utility/macros.h"

enum reg_index
{
	RI_CCR,
	RI_END,
};

REGDEF_BLOCK_BEGIN()
	REG_R(18);
	REG_K32(PRESC,4);
	REG_B32(VFREFEN);
	REG_B32(TSEN);
	REG_B32(VBATEN);
	REG_R(7);
REGDEF_BLOCK_END(adcc, ccr);

static const stm32_reginfo_t stm32f030_adcc_reginfo[RI_END] =
{
	[RI_CCR] = {.mask = 0x00C00000, .unimp_mask = 0x00C00000},

};

static const stm32_reginfo_t stm32g070_adcc_reginfo[RI_END] =
{
	[RI_CCR] = {.mask = 0x1FC0000, .unimp_mask = 0x1C00000 },
};

 static const stm32_periph_variant_t stm32_adcc_variants[2] = {
 	{TYPE_STM32F030_ADCC, stm32f030_adcc_reginfo},
 	{TYPE_STM32G070_ADCC, stm32g070_adcc_reginfo},
 };

typedef struct COM_CLASS_NAME(Adcc) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Adcc);


typedef struct COM_STRUCT_NAME(Adcc) {
    /* <private> */
    STM32Peripheral parent_obj;

    /* <public> */
    MemoryRegion mmio;

    union {
        uint32_t raw; // Common registers. There's only 3 but #4 is used for initialization/sanity.
        struct {
            REGDEF_NAME(adcc, ccr) CCR;
        } defs;
    } regs ;

	stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Adcc);


static void stm32_common_adcc_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(Adcc) *s = STM32COM_ADCC(dev);
    memset(&s->regs,0,sizeof(s->regs));
}

extern uint16_t stm32_common_adcc_get_adcpre(COM_STRUCT_NAME(Adcc) *s)
{
	static uint16_t divisors[] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256, 0, 0, 0, 0};
    return divisors[s->regs.defs.CCR.PRESC];
}

static uint64_t stm32_common_adcc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
	COM_STRUCT_NAME(Adcc) *s = STM32COM_ADCC(opaque);

	uint32_t offset = addr&0x3;
    addr>>=2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32COMMON ADCC"); // LCOV_EXCL_LINE

	uint32_t data = s->regs.raw;
    switch (addr) {
        case RI_CCR:
			break;
        default:
            return 0;
    }

	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, offset, 0b100);
	return data;
}

static void stm32_common_adcc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
	COM_STRUCT_NAME(Adcc) *s = STM32COM_ADCC(opaque);

	uint32_t offset = addr&0x3;
    addr>>=2; // Get index in array.

	CHECK_BOUNDS_W(addr, value, RI_END, s->reginfo, "STM32Common ADCC"); // LCOV_EXCL_LINE


    switch (addr) {
        case RI_CCR:
			ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw, value, size, offset, 0b100);
            s->regs.raw = value;
            break;
    }
}

static const MemoryRegionOps stm32common_adcc_ops = {
    .read = stm32_common_adcc_read,
    .write = stm32_common_adcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};


static const VMStateDescription vmstate_stm32common_adcc = {
    .name = TYPE_STM32COM_ADCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(regs.raw, COM_STRUCT_NAME(Adcc)),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_common_adcc_init(Object *obj)
{
    COM_STRUCT_NAME(Adcc) *s = STM32COM_ADCC(obj);

    // Check the register union definitions... This thows compile errors if they are misaligned, so it's ok in regards to not throwing exceptions
    // during object init in QEMU.
    CHECK_ALIGN(sizeof(s->regs.defs),sizeof(uint32_t)*RI_END, "overall sizw");
    CHECK_ALIGN(sizeof(s->regs.defs),sizeof(s->regs.raw), "Raw array");
    CHECK_REG_u32(s->regs.defs.CCR);

    STM32_MR_IO_INIT(&s->mmio, obj, &stm32common_adcc_ops, s, 4U*(RI_END));
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
	COM_CLASS_NAME(Adcc) *k = STM32COM_ADCC_GET_CLASS(obj);
	s->reginfo = k->var_reginfo;

}

static void stm32_common_adcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32_common_adcc_reset;
    dc->vmsd = &vmstate_stm32common_adcc;

	COM_CLASS_NAME(Adcc) *k = STM32COM_ADCC_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo stm32_common_adcc_info = {
    .name          = TYPE_STM32COM_ADCC,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Adcc)),
	.class_size    = sizeof(COM_CLASS_NAME(Adcc)),
	.abstract = true,

};

static void stm32_common_adcc_register_types(void)
{
    type_register_static(&stm32_common_adcc_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_adcc_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_adcc_variants[i].variant_name,
            .parent     = TYPE_STM32COM_ADCC,
			.instance_init = stm32_common_adcc_init,
			.class_init    = stm32_common_adcc_class_init,
            .class_data = (void *)stm32_adcc_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}


type_init(stm32_common_adcc_register_types)
