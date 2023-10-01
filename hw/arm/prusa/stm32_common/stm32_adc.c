/*
 * STM32 Common ADC
 * Layout is known used by the following chips:
 * STM32F030x
 * STM32G070 (with extra registers)
 *
 * Copyright (c) 2021-3 VintagePC <http://github.com/vintagepc>
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
#include "hw/qdev-properties.h"
#include "qemu/typedefs.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "stm32_adcc.h"
#include "stm32_rcc_if.h"
#include "stm32_adc_regdata.h"

#ifndef STM_ADC_ERR_DEBUG
#define STM_ADC_ERR_DEBUG 0
#endif

#define STM32_COM_ADC_MAX_REG_CHANNELS 19

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Adc), COM_CLASS_NAME(Adc), STM32COM_ADC);

REGDEF_BLOCK_BEGIN()
	REG_B32(ADRDY);
	REG_B32(EOSMP);
	REG_B32(EOC);
	REG_B32(EOSEQ);
	REG_B32(OVR);
	REG_R(2);
	REG_B32(AWD);
	REG_R(24);
REGDEF_BLOCK_END(adc, isr)

REGDEF_BLOCK_BEGIN()
	REG_B32(ADRDYIE);
	REG_B32(EOSMPIE);
	REG_B32(EOCIE);
	REG_B32(EOSEQIE);
	REG_B32(OVRIE);
	REG_R(2);
	REG_B32(AWDIE);
	REG_R(24);
REGDEF_BLOCK_END(adc, ier)

REGDEF_BLOCK_BEGIN()
	REG_B32(ADEN);
	REG_B32(ADDIS);
	REG_B32(ADSTART);
	REG_RB();
	REG_B32(ADSTP);
	REG_R(23);
	REG_B32(ADVREGEN);
	REG_R(2);
	REG_B32(ADCAL);
REGDEF_BLOCK_END(adc, cr)

REGDEF_BLOCK_BEGIN();
	REG_B32(DMAEN);
	REG_B32(DMACFG);
	REG_B32(SCANDIR);
	REG_K32(RES, 2);
	REG_B32(ALIGN);
	REG_K32(EXTSEL, 3);
	REG_RB();
	REG_K32(EXTEN,2);
	REG_B32(OVRMOD);
	REG_B32(CONT);
	REG_B32(WAIT);
	REG_B32(AUTOFF);
	REG_B32(DISCEN);
	REG_R(5);
	REG_B32(AWDSGL);
	REG_B32(AWDEN);
	REG_R(2);
	REG_K32(AWDCH, 5);
	REG_RB();
REGDEF_BLOCK_END(adc, cfgr1)

REGDEF_BLOCK_BEGIN()
	REG_B32(OVSE);
	REG_RB();
	REG_K32(OVSR, 3);
	REG_K32(OVSS, 4);
	REG_R(21);
	REG_K32(CKMODE, 2);
REGDEF_BLOCK_END(adc, cfgr2)

REGDEF_BLOCK_BEGIN()
	REG_K32(SMP1,3);
	REG_RB();
	REG_K32(SMP2, 3);
	REG_RB();
	REG_K32(SMPSEL,19);
	REG_R(5);
REGDEF_BLOCK_END(adc, smpr)

REGDEF_BLOCK_BEGIN()
	REG_K32(LT, 12);
	REG_R(4);
	REG_K32(HT,12);
	REG_R(4);
REGDEF_BLOCK_END(adc, tr)

REGDEF_BLOCK_BEGIN()
	REG_B32(CHSEL0);
	REG_B32(CHSEL1);
	REG_B32(CHSEL2);
	REG_B32(CHSEL3);
	REG_B32(CHSEL4);
	REG_B32(CHSEL5);
	REG_B32(CHSEL6);
	REG_B32(CHSEL7);
	REG_B32(CHSEL8);
	REG_B32(CHSEL9);
	REG_B32(CHSEL10);
	REG_B32(CHSEL11);
	REG_B32(CHSEL12);
	REG_B32(CHSEL13);
	REG_B32(CHSEL14);
	REG_B32(CHSEL15);
	REG_B32(CHSEL16);
	REG_B32(CHSEL17);
	REG_B32(CHSEL18);
	REG_R(13);
REGDEF_BLOCK_END(adc, chselr)

REGDEF_BLOCK_BEGIN()
	REG_R(22);
	REG_B32(VREFEN);
	REG_B32(TSEN);
	REG_R(8);
REGDEF_BLOCK_END(adc, ccr)

enum smpr_table {
	SMPR_UNKNOWN,
	SMPR_F030,
	SMPR_G070,
};

typedef struct COM_STRUCT_NAME(Adc) {
	/* <private> */
	STM32Peripheral parent;

	/* <public> */
	MemoryRegion mmio;

	union {
		struct {
			REGDEF_NAME(adc, isr) ISR;		//0x00
			REGDEF_NAME(adc, ier) IER;		//0x04
			REGDEF_NAME(adc, cr) CR;		//0x08
			REGDEF_NAME(adc, cfgr1) CFGR1;	//0x0C
			REGDEF_NAME(adc, cfgr2) CFGR2;	//0x10
			REGDEF_NAME(adc, smpr)  SMPR;	//0x14
			REGDEF_R(0x18);
			REGDEF_R(0x1C);
			REGDEF_NAME(adc, tr) AWD1TR;		//0x20
			REGDEF_NAME(adc, tr) AWD2TR;		//0x24
			REGDEF_NAME(adc, chselr) CHSELR;	//0x28
			REGDEF_NAME(adc, tr) AWD3TR;		//0x2C
			REGDEF_R(0x30);
			REGDEF_R(0x34);
			REGDEF_R(0x38);
			REGDEF_R(0x3C);
			REG_S32(DR, 16);				//0x40
		} QEMU_PACKED defs;
		uint32_t raw[RI_END];
	} regs;

	qemu_irq irq;
	qemu_irq irq_read[STM32_COM_ADC_MAX_REG_CHANNELS]; // Set when the ADC wants to get a value from the channel.

	int adc_data[STM32_COM_ADC_MAX_REG_CHANNELS]; // Store the peripheral data received.
	QEMUTimer* next_eoc;

	uint8_t adc_sequence_position;

	int smpr_table;

	COM_STRUCT_NAME(Adcc) *adcc;

	stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Adc);


static const stm32_reginfo_t stm32f030_adc_reginfo[RI_END] =
{
	[RI_ISR] = {.mask = 0b10011111 },
	[RI_IER] = {.mask = 0b10011111 },
	[RI_CR] = {.mask = 		0x80000017},
	[RI_CFGR1] = {.mask = 	0x7CC1FDFF},
	[RI_CFGR2] = {.mask = 	0xC0000000, .unimp_mask = 0xC0000000 },
	[RI_SMPR] = {.mask = 0x7},
	[(RI_SMPR +1U)... (RI_AWD1TR - 1U)] = {.is_reserved = true},
	[RI_AWD1TR] = {.mask = 0x0FFF0FFF, .reset_val = 0x0FFF0000, .unimp_mask = UINT32_MAX},
	[RI_AWD2TR] = {.is_reserved = true},
	[RI_CHSELR] {.mask = 0x3FFFF},
	[(RI_CHSELR + 1U) ... (RI_DR - 1U)] = {.is_reserved = true},
	[RI_DR] = {.mask = UINT16_MAX}
};

static const stm32_reginfo_t stm32g070_adc_reginfo[RI_END] =
{
	[RI_ISR] = {.mask = 0b10011111 },
	[RI_IER] = {.mask = 0b10011111 },
	[RI_CR] = {.mask = 		0x80000017},
	[RI_CFGR1] = {.mask = 	0x7CC1FDFF},
	[RI_CFGR2] = {.mask = 	0xE00003FD, .unimp_mask = 0xC00001FD},
	[RI_SMPR] = {.mask = 	0x07FFFF77},
	[(RI_SMPR +1U)... (RI_AWD1TR - 1U)] = {.is_reserved = true},
	[RI_AWD1TR] = {.mask = 0x0FFF0FFF, .reset_val = 0x0FFF0000, .unimp_mask = UINT32_MAX},
	[RI_AWD2TR] = {.mask = 0x0FFF0FFF, .reset_val = 0x0FFF0000, .unimp_mask = UINT32_MAX},
	[RI_CHSELR] {.mask = 0x7FFFF},
	[RI_AWD3TR] = {.mask = 0x0FFF0FFF, .reset_val = 0x0FFF0000, .unimp_mask = UINT32_MAX},
	[(RI_AWD3TR + 1U) ... (RI_DR - 1U)] = {.is_reserved = true},
	[RI_DR] = {.mask = UINT16_MAX}
};

 static const stm32_periph_variant_t stm32_adc_variants[2] = {
 	{TYPE_STM32F030_ADC, stm32f030_adc_reginfo},
 	{TYPE_STM32G070_ADC, stm32g070_adc_reginfo},
};

typedef struct COM_CLASS_NAME(Adc) {
	STM32PeripheralClass parent_class;
	stm32_reginfo_t var_reginfo[RI_END];
	int smpr_table;
} COM_CLASS_NAME(Adc);

#define DB_PRINT_L(lvl, fmt, args...) do { \
	if (STM_ADC_ERR_DEBUG >= lvl) { \
		qemu_log("%s: " fmt, __func__, ## args); \
	} \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32_adc_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(dev);

   	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
	s->adc_sequence_position = 0;
	memset(&s->adc_data,0,STM32_COM_ADC_MAX_REG_CHANNELS*sizeof(int));
	if (s->next_eoc)
		timer_del(s->next_eoc);
}

// Technically these should all include half a nanosecond but
// to keep it as integer math we include the .5ns from the conversion time here.
static uint16_t f030_2x_smpr[] = { 2, 8, 14, 29, 42, 56, 72, 240 };
static uint16_t g070_2x_smpr[] = { 2, 4,  8, 13, 20, 40, 80, 161 };

static uint8_t align_shifts[4] = { 4, 6, 8, 2};

static uint16_t adc_lookup_smpr(COM_STRUCT_NAME(Adc) *s, uint8_t value) {
	assert(value < 8);
	switch(s->smpr_table)
	{
		case SMPR_G070:
			return g070_2x_smpr[value];
		case SMPR_F030:
			return f030_2x_smpr[value];
		default: // LCOV_EXCL_LINE
			g_assert_not_reached(); // LCOV_EXCL_LINE
	}
}

static uint32_t stm32_adc_get_value(COM_STRUCT_NAME(Adc) *s)
{
	uint32_t internal_value = s->adc_data[s->adc_sequence_position];
	// I'm not sure why this is yet - some sort of built in oversampling
	// that is enabled in non-DMA mode?
	// Mask: RES 0..3 == 12..6 bit mask, so shift right 2* RES
	internal_value >>= (s->regs.defs.CFGR1.RES<<1); // Raw value is max 12 bits, with up to 20 bits internal OVS for a final 16 bit value.

	if (!s->regs.defs.CFGR1.DMAEN) {
		if (s->regs.defs.CFGR2.OVSE)
		{
			internal_value*= (1U << (s->regs.defs.CFGR2.OVSR + 1));
			internal_value >>= s->regs.defs.CFGR2.OVSS;
		}
		else // Legacy code for the F0? leaving in place for now but it's probably wrong even though it sorta works...
		{
			bool rate_sel = (s->regs.defs.SMPR.SMPSEL >> s->adc_sequence_position) & 1U;
			s->regs.defs.DR*=(adc_lookup_smpr(s,
				rate_sel ? s->regs.defs.SMPR.SMP2 : s->regs.defs.SMPR.SMP1
			));
		}
	}
	s->regs.defs.DR = internal_value & UINT16_MAX;
	//printf("ADC DR read for chan %u (%u)\n", s->adc_sequence_position, s->regs.defs.DR);
	if (s->regs.defs.CFGR1.ALIGN && !s->regs.defs.CFGR2.OVSE) {
		return (s->regs.defs.DR << align_shifts[s->regs.defs.CFGR1.RES]);
	} else {
		return s->regs.defs.DR;
	}
}

// ADC data in from peripherals
static void stm32_adc_data_in(void *opaque, int n, int level){
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);
	s->adc_data[n] = level;
	// printf("ADC: Ch %d new data: %d\n",n, level);
}

static void stm32_adc_schedule_next(COM_STRUCT_NAME(Adc) *s) {
	if (!s->regs.defs.CR.ADEN || s->regs.defs.CHSELR.raw == 0 || !stm32_rcc_if_check_periph_clk(&s->parent))
	{
		return;
	}

	// Calculate the clock rate
	uint64_t clock = stm32_rcc_if_get_periph_freq(&s->parent);

	if (s->adcc)
	{
		clock /= stm32_common_adcc_get_adcpre(s->adcc);
	}

	// #bits:
	uint32_t conv_cycles = (12U - (s->regs.defs.CFGR1.RES<<1U));
	bool rate_sel = (s->regs.defs.SMPR.SMPSEL >> s->adc_sequence_position) & 1U;
	conv_cycles += (adc_lookup_smpr(s,
			rate_sel? s->regs.defs.SMPR.SMP2 : s->regs.defs.SMPR.SMP1
		));

	uint64_t delay_ns = (1000000000UL * conv_cycles) / clock;
	// printf("ADC conversion: %u cycles @ %"PRIu64" Hz (%lu nSec)\n", conv_cycles, clock, delay_ns);
	timer_mod_ns(s->next_eoc, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+delay_ns);

}


static uint64_t stm32_adc_read(void *opaque, hwaddr addr,
									 unsigned int size)
{
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);

   // DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);
	int offset = addr&0x3;
	addr>>=2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "Common ADC"); // LCOV_EXCL_LINE

	uint32_t data = s->regs.raw[addr];

	switch (addr) {
	case RI_ISR ... RI_CHSELR:
		break;
	case RI_DR:
		if (s->regs.defs.CR.ADEN && s->regs.defs.ISR.EOC) {
			s->regs.defs.ISR.EOC ^= s->regs.defs.ISR.EOC;
			data = stm32_adc_get_value(s);
		} else {
			qemu_log_mask(LOG_GUEST_ERROR, "Read ADC while conversion not ready!\n");
			return 0;
		}
		break;
	default:
		return 0;
	}
	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, offset, 0b111);
	return data;
}

static void stm32_adc_convert(COM_STRUCT_NAME(Adc) *s)
{
	qemu_irq_pulse(s->irq_read[s->adc_sequence_position]); // Toggle the data read request IRQ. The receiver can opt to send a new value (or do nothing)
}

static void stm32_adc_update_irqs(COM_STRUCT_NAME(Adc) *s, int level) {

	bool bChanged = level^s->regs.defs.ISR.EOC;
	s->regs.defs.ISR.EOC = level;

	if (s->regs.defs.IER.EOCIE && level && bChanged)
	{
		qemu_irq_raise(s->irq);
	}
}

static void stm32_adc_set_next_channel(COM_STRUCT_NAME(Adc) *s)
{
	uint32_t chanmask = s->regs.defs.CHSELR.raw >> (s->adc_sequence_position + 1U);
	s->adc_sequence_position++;
	while (!(chanmask & 0x1))
	{
		s->adc_sequence_position++;
		chanmask >>= 1;
		if (s->adc_sequence_position > STM32_COM_ADC_MAX_REG_CHANNELS) // Looped, we went off the end.
		{
			s->adc_sequence_position = 0;
			chanmask = s->regs.defs.CHSELR.raw;
		}
	}
	// printf("Next ADC channel: %u\n", s->adc_sequence_position);
}

static void stm32_adc_eoc_deadline(void *opaque) {

	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);
	if (!s->regs.defs.CFGR1.DMAEN) // This is probably wrong, this should be called regardless, but the old version of doing it after the convert was working...
	{
		stm32_adc_set_next_channel(s);
	}
	stm32_adc_convert(s);
	if (s->regs.defs.IER.EOCIE || s->adc_sequence_position == 0)
	{
		// Either end of cycle or end-of-sequence.
		stm32_adc_update_irqs(s, 1);
	}

	if (s->regs.defs.CFGR1.DMAEN)
	{
		s->regs.defs.ISR.EOC = 1;
		qemu_set_irq(s->parent.dmar[DMAR_P2M], s->mmio.addr + (4U*RI_DR));
		if (s->regs.defs.CFGR1.CONT && s->regs.defs.CHSELR.raw)
		{
			stm32_adc_set_next_channel(s);
			if (!s->regs.defs.CFGR1.WAIT) // don't schedule if in WAIT mode.
			{
				stm32_adc_schedule_next(s);
			}
		}
	}
	else if (!s->regs.defs.CFGR1.CONT && s->regs.defs.CFGR1.EXTEN == 0)
	{
		if (s->regs.defs.CFGR1.DISCEN)
		{
			printf("FIXME - unimplemented DISCEN EOC mode\n");
		}
		else
		{
			s->regs.defs.CR.ADSTART = 0; // Stop the ADC, single conversion.
			s->regs.defs.ISR.EOC = 1;
		}
	}
}

static void stm32_adc_write(void *opaque, hwaddr addr,
					   uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);

	// printf("ADC_write %d : 0x%" HWADDR_PRIx ", Value: 0x%x\n",
//             s->id, addr, (uint32_t)value);

	uint8_t offset = addr&0x3;
	addr>>=2; // Get index in array.
	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "Common ADC"); // LCOV_EXCL_LINE
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);

	if (addr >= 0x100) {
		qemu_log_mask(LOG_UNIMP,
					  "%s: ADC Common Register Unsupported\n", __func__);
	}

	switch (addr) {
		case RI_CFGR1:
		case RI_CFGR2:
		case RI_IER:
		case RI_AWD1TR:
		case RI_AWD2TR:
		case RI_AWD3TR:
		case RI_SMPR:
		case RI_CHSELR:
			s->regs.raw[addr] = data;
			break;
		case RI_CR:
		{
			REGDEF_NAME(adc, cr) new = {.raw = data };
			// Most new bits are RS, except VREGEN.
			s->regs.raw[addr] |= data;
			s->regs.defs.CR.ADVREGEN = new.ADVREGEN;
			if (s->regs.defs.CR.ADEN && !s->regs.defs.CR.ADDIS)
			{
				s->regs.defs.ISR.ADRDY = 1;
			}
			if (s->regs.defs.CR.ADSTART)
			{
				stm32_adc_schedule_next(s);
			}
			if (s->regs.defs.CR.ADCAL)
			{
				s->regs.defs.CR.ADCAL = 0; // No calibration required :)
			}
		}
			break;
		default:
			return;
	}
}

static Property stm32common_adc_properties[] = {
	DEFINE_PROP_LINK("adcc", COM_STRUCT_NAME(Adc), adcc, TYPE_STM32COM_ADCC, COM_STRUCT_NAME(Adcc) *),
	DEFINE_PROP_END_OF_LIST()
};

static const MemoryRegionOps stm32_adc_ops = {
	.read = stm32_adc_read,
	.write = stm32_adc_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.impl.min_access_size = 4,
	.impl.max_access_size = 4,
};


static const VMStateDescription vmstate_stm32_adc = {
	.name = TYPE_STM32COM_ADC,
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Adc), RI_END),
		VMSTATE_INT32_ARRAY(adc_data,COM_STRUCT_NAME(Adc), STM32_COM_ADC_MAX_REG_CHANNELS),
		VMSTATE_UINT8(adc_sequence_position,COM_STRUCT_NAME(Adc)),
		VMSTATE_END_OF_LIST()
	}
};

static void stm32_adc_init(Object *obj)
{
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(obj);

	// Check the register union definitions... This thows compile errors if they are misaligned, so it's ok in regards to not throwing exceptions
	// during object init in QEMU.
	CHECK_ALIGN(sizeof(s->regs.raw),sizeof(uint32_t)*RI_END, "defs union");
	CHECK_ALIGN(sizeof(s->regs.defs),sizeof(s->regs.raw), "Raw array");
	// Check the bitfields. S32s should be fine because
	// the macro handles the padding math and problems are detected by the overall size change above
	CHECK_REG_u32(s->regs.defs.ISR);
	CHECK_REG_u32(s->regs.defs.IER);
	CHECK_REG_u32(s->regs.defs.CR);
	CHECK_REG_u32(s->regs.defs.CFGR1);
	CHECK_REG_u32(s->regs.defs.CFGR2);
	CHECK_REG_u32(s->regs.defs.AWD1TR);
	CHECK_REG_u32(s->regs.defs.AWD2TR);
	CHECK_REG_u32(s->regs.defs.AWD3TR);
	CHECK_REG_u32(s->regs.defs.CHSELR);
	CHECK_UNION(COM_STRUCT_NAME(Adc), regs.defs.CHSELR, regs.raw[RI_CHSELR]);
	QEMU_BUILD_BUG_MSG(RI_END != 17, "Size of register array has changed. You need to update VMState!");


	s->next_eoc = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_adc_eoc_deadline, s);


	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

	qdev_init_gpio_out_named(DEVICE(obj), s->irq_read, "adc_read", STM32_COM_ADC_MAX_REG_CHANNELS);

	qdev_init_gpio_in_named(DEVICE(obj),stm32_adc_data_in, "adc_data_in", STM32_COM_ADC_MAX_REG_CHANNELS);

	STM32_MR_IO_INIT(&s->mmio, obj, &stm32_adc_ops, s, 512);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
	COM_CLASS_NAME(Adc) *k = STM32COM_ADC_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
	s->smpr_table = k->smpr_table;
}

static void stm32_adc_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	dc->reset = stm32_adc_reset;
	dc->vmsd = &vmstate_stm32_adc;

	device_class_set_props(dc, stm32common_adc_properties);

	COM_CLASS_NAME(Adc) *k = STM32COM_ADC_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	if (data == stm32f030_adc_reginfo)
	{
		k->smpr_table = SMPR_F030;
	}
	else if (data == stm32g070_adc_reginfo)
	{
		k->smpr_table = SMPR_G070;
	}
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo stm32_common_adc_info = {
	.name          = TYPE_STM32COM_ADC,
	.parent        = TYPE_STM32_PERIPHERAL,
	.instance_size = sizeof(COM_STRUCT_NAME(Adc)),
	.class_size    = sizeof(COM_CLASS_NAME(Adc)),
	.abstract = true,
};

static void stm32_adc_register_types(void)
{
	type_register_static(&stm32_common_adc_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_adc_variants); ++i) {
		TypeInfo ti = {
			.name       = stm32_adc_variants[i].variant_name,
			.parent     = TYPE_STM32COM_ADC,
			.instance_init = stm32_adc_init,
			.class_init    = stm32_adc_class_init,
			.class_data = (void *)stm32_adc_variants[i].variant_regs,
		};
		type_register(&ti);
	}
}

type_init(stm32_adc_register_types)
