/*
 * STM32 Common ADC
 * Layout is known used by the following chips:
 * STM32F030x
 * STM32G070 (with extra registers)
 *
 * Copyright (c) 2021-4 VintagePC <http://github.com/vintagepc>
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

typedef union {
	struct {
		uint32_t ADRDY         : 1; // /*!< ADC ready flag */
		uint32_t EOSMP         : 1; // /*!< ADC group regular end of sampling flag */
		uint32_t EOC           : 1; // /*!< ADC group regular end of unitary conversion flag */
		uint32_t EOS           : 1; // /*!< ADC group regular end of sequence conversions flag */
		uint32_t OVR           : 1; // /*!< ADC group regular overrun flag */
		uint32_t JEOC          : 1; // /*!< ADC group injected end of unitary conversion flag */
		uint32_t JEOS          : 1; // /*!< ADC group injected end of sequence conversions flag */
		uint32_t AWD1          : 1; // /*!< ADC analog watchdog 1 flag */
		uint32_t AWD2          : 1; // /*!< ADC analog watchdog 2 flag */
		uint32_t AWD3          : 1; // /*!< ADC analog watchdog 3 flag */
		uint32_t JQOVF         : 1; // /*!< ADC group injected contexts queue overflow flag */
		uint32_t _reserved11   :21;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,int);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,int),bits);

typedef union {
	struct {
		uint32_t ADEN          : 1; // /*!< ADC enable */
		uint32_t ADDIS         : 1; // /*!< ADC disable */
		uint32_t ADSTART       : 1; // /*!< ADC group regular conversion start */
		uint32_t JADSTART      : 1; // /*!< ADC group injected conversion start */
		uint32_t ADSTP         : 1; // /*!< ADC group regular conversion stop */
		uint32_t JADSTP        : 1; // /*!< ADC group injected conversion stop */
		uint32_t _reserved6    :22;
		uint32_t ADVREGEN      : 1; // /*!< ADC voltage regulator enable */
		uint32_t DEEPPWD       : 1; // /*!< ADC deep power down enable */
		uint32_t ADCALDIF      : 1; // /*!< ADC differential mode for calibration */
		uint32_t ADCAL         : 1; // /*!< ADC calibration */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,cr);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,cr),bits);

typedef union {
	struct {
		uint32_t DMAEN         : 1; // /*!< ADC DMA transfer enable */
		uint32_t DMACFG        : 1; // /*!< ADC DMA transfer configuration */
		uint32_t SCANDIR       : 1; // /*!< ADC group regular sequencer scan direction */
		uint32_t RES           : 2; // /*!< ADC data resolution */
		uint32_t _reserved6	   : 5; // variant-specific
		uint32_t EXTEN         : 2; // /*!< ADC group regular external trigger polarity */
		uint32_t OVRMOD        : 1; // /*!< ADC group regular overrun configuration */
		uint32_t CONT          : 1; // /*!< ADC group regular continuous conversion mode */
		uint32_t _reserved14   : 2; // WAIT/AUTOFF or AUTDLY/ALIGN
		uint32_t DISCEN        : 1; // /*!< ADC group regular sequencer discontinuous mode */
		uint32_t DISCNUM       : 3; // /*!< ADC group regular sequencer discontinuous number of ranks */
		uint32_t JDISCEN       : 1; // /*!< ADC group injected sequencer discontinuous mode */
		uint32_t _reserved17   : 1;
		uint32_t AWD1SGL       : 1; // /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
		uint32_t AWD1EN        : 1; // /*!< ADC analog watchdog 1 enable on scope ADC group regular */
		uint32_t JAWD1EN       : 1; // /*!< ADC analog watchdog 1 enable on scope ADC group injected */
		uint32_t JAUTO         : 1; // /*!< ADC group injected automatic trigger mode */
		uint32_t AWD1CH        : 5; // /*!< ADC analog watchdog 1 monitored channel selection */
		uint32_t JQDIS         : 1; // /*!< ADC group injected contexts queue disable */
	} QEMU_PACKED bits;
	uint32_t raw;
	}  REGDEF_NAME(adc,com_cfgr1);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,com_cfgr1),bits);

typedef union {
	struct {
		uint32_t _common1      : 5; // See common def (shared bits)
		uint32_t ALIGN         : 1; // /*!< ADC data alignment */
		uint32_t EXTSEL        : 3; // /*!< ADC group regular external trigger source */
		uint32_t _reserved9    : 1;
		uint32_t _common10     : 4; //
		uint32_t WAIT          : 1; // /*!< ADC low power auto wait */
		uint32_t AUTOFF        : 1; // /*!< ADC low power auto power off */
		uint32_t _common16     : 5;
		uint32_t CHSELRMOD     : 1; // /*!< ADC group regular sequencer mode */
		uint32_t _common22     :10;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(f_g_adc,cfgr1);
CHECK_TYPEDEF_u32(REGDEF_NAME(f_g_adc,cfgr1),bits);

typedef union {
	struct {
		uint32_t _common1      : 5; // See common def (shared bits)
		uint32_t EXTSEL        : 5; // /*!< ADC group regular external trigger source */
		uint32_t _common10     : 4; //
		uint32_t AUTDLY        : 1; // /*!< ADC low power auto wait */
		uint32_t ALIGN         : 1; // /*!< ADC data alignment */
		uint32_t _common16     : 5;
		uint32_t JQM           : 1; // /*!< ADC group injected contexts queue mode */
		uint32_t _common22     :10;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(h_adc,cfgr1);
CHECK_TYPEDEF_u32(REGDEF_NAME(h_adc,cfgr1),bits);

typedef union {
	struct {
		uint32_t ROVSE         : 1; // /*!< ADC oversampler enable on scope ADC group regular */
		uint32_t JOVSE         : 1; // /*!< ADC oversampler enable on scope ADC group injected */
		uint32_t OVSR          : 3; // /*!< ADC oversampling ratio */
		uint32_t OVSS          : 4; // /*!< ADC oversampling shift */
		uint32_t TROVS         : 1; // /*!< ADC oversampling discontinuous mode (triggered mode) for ADC group regular */
		uint32_t ROVSM         : 1; // /*!< ADC oversampling mode managing interlaced conversions of ADC group regular and group injected */
		uint32_t _reserved11   : 5;
		uint32_t GCOMP         : 1; // /*!< ADC Gain Compensation mode */
		uint32_t _reserved17   : 8;
		uint32_t SWTRIG        : 1; // /*!< ADC Software Trigger Bit for Sample time control trigger mode */
		uint32_t BULB          : 1; // /*!< ADC Bulb sampling mode */
		uint32_t SMPTRIG       : 1; // /*!< ADC Sample Time Control Trigger mode */
		uint32_t _reserved28   : 1;
		uint32_t LFTRIG        : 1; // /*!< ADC Low Frequency Trigger */
		uint32_t CKMODE  	   : 2;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,cfgr2);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,cfgr2),bits);

typedef union {
	struct {
		uint32_t SMP1          : 3; // /*!< ADC group of channels sampling time 1 */
		uint32_t _reserved3    : 1;
		uint32_t SMP2          : 3; // /*!< ADC group of channels sampling time 2 */
		uint32_t _reserved7    : 1;
		uint32_t SMPSEL        :19; // /*!< ADC channel 0 sampling time selection *//
		uint32_t _reserved27   : 5;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(f_g_adc,smpr);
CHECK_TYPEDEF_u32(REGDEF_NAME(f_g_adc,smpr),bits);

typedef union {
	struct {
		uint32_t SMP0          : 3; // /*!< ADC channel 0 sampling time selection  */
		uint32_t SMP1          : 3; // /*!< ADC channel 1 sampling time selection  */
		uint32_t SMP2          : 3; // /*!< ADC channel 2 sampling time selection  */
		uint32_t SMP3          : 3; // /*!< ADC channel 3 sampling time selection  */
		uint32_t SMP4          : 3; // /*!< ADC channel 4 sampling time selection  */
		uint32_t SMP5          : 3; // /*!< ADC channel 5 sampling time selection  */
		uint32_t SMP6          : 3; // /*!< ADC channel 6 sampling time selection  */
		uint32_t SMP7          : 3; // /*!< ADC channel 7 sampling time selection  */
		uint32_t SMP8          : 3; // /*!< ADC channel 8 sampling time selection  */
		uint32_t SMP9          : 3; // /*!< ADC channel 9 sampling time selection  */
		uint32_t _reserved30   : 1;
		uint32_t SMPPLUS       : 1; // /*!< ADC channels sampling time additional setting */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(h_adc,smpr);
CHECK_TYPEDEF_u32(REGDEF_NAME(h_adc,smpr),bits);

typedef union {
	struct {
		uint32_t SMP10         : 3; // /*!< ADC channel 10 sampling time selection  */
		uint32_t SMP11         : 3; // /*!< ADC channel 11 sampling time selection  */
		uint32_t SMP12         : 3; // /*!< ADC channel 12 sampling time selection  */
		uint32_t SMP13         : 3; // /*!< ADC channel 13 sampling time selection  */
		uint32_t SMP14         : 3; // /*!< ADC channel 14 sampling time selection  */
		uint32_t SMP15         : 3; // /*!< ADC channel 15 sampling time selection  */
		uint32_t SMP16         : 3; // /*!< ADC channel 16 sampling time selection  */
		uint32_t SMP17         : 3; // /*!< ADC channel 17 sampling time selection  */
		uint32_t SMP18         : 3; // /*!< ADC channel 18 sampling time selection  */
		uint32_t _reserved27   : 5;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,smpr2);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,smpr2),bits);

typedef union {
	struct {
		uint32_t L             : 4; // /*!< ADC group regular sequencer scan length */
		uint32_t _reserved4    : 2;
		uint32_t SQ1           : 5; // /*!< ADC group regular sequencer rank 1 */
		uint32_t _reserved11   : 1;
		uint32_t SQ2           : 5; // /*!< ADC group regular sequencer rank 2 */
		uint32_t _reserved17   : 1;
		uint32_t SQ3           : 5; // /*!< ADC group regular sequencer rank 3 */
		uint32_t _reserved23   : 1;
		uint32_t SQ4           : 5; // /*!< ADC group regular sequencer rank 4 */
		uint32_t _reserved29   : 3;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,sqr1);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,sqr1),bits);

typedef union {
	struct {
		uint32_t SQ5           : 5; // /*!< ADC group regular sequencer rank 5 */
		uint32_t _reserved5    : 1;
		uint32_t SQ6           : 5; // /*!< ADC group regular sequencer rank 6 */
		uint32_t _reserved11   : 1;
		uint32_t SQ7           : 5; // /*!< ADC group regular sequencer rank 7 */
		uint32_t _reserved17   : 1;
		uint32_t SQ8           : 5; // /*!< ADC group regular sequencer rank 8 */
		uint32_t _reserved23   : 1;
		uint32_t SQ9           : 5; // /*!< ADC group regular sequencer rank 9 */
		uint32_t _reserved29   : 3;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,sqr2);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,sqr2),bits);

typedef union {
	struct {
		uint32_t SQ10          : 5; // /*!< ADC group regular sequencer rank 10 */
		uint32_t _reserved5    : 1;
		uint32_t SQ11          : 5; // /*!< ADC group regular sequencer rank 11 */
		uint32_t _reserved11   : 1;
		uint32_t SQ12          : 5; // /*!< ADC group regular sequencer rank 12 */
		uint32_t _reserved17   : 1;
		uint32_t SQ13          : 5; // /*!< ADC group regular sequencer rank 13 */
		uint32_t _reserved23   : 1;
		uint32_t SQ14          : 5; // /*!< ADC group regular sequencer rank 14 */
		uint32_t _reserved29   : 3;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,sqr3);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,sqr3),bits);

typedef union {
	struct {
		uint32_t SQ15          : 5; // /*!< ADC group regular sequencer rank 15 */
		uint32_t _reserved5    : 1;
		uint32_t SQ16          : 5; // /*!< ADC group regular sequencer rank 16 */
		uint32_t _reserved11   :21;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(adc,sqr4);
CHECK_TYPEDEF_u32(REGDEF_NAME(adc,sqr4),bits);

REGDEF_BLOCK_BEGIN()
	REG_K32(LT, 12);
	REG_R(4);
	REG_K32(HT,12);
	REG_R(4);
REGDEF_BLOCK_END(adc, tr)


enum type_table {
	TYPE_UNKNOWN,
	TYPE_F030,
	TYPE_G070,
	TYPE_H503,
};

typedef struct COM_STRUCT_NAME(Adc) {
	/* <private> */
	STM32Peripheral parent;

	/* <public> */
	MemoryRegion mmio;

	union {
		struct {
			REGDEF_NAME(adc, int) ISR;		//0x00
			REGDEF_NAME(adc, int) IER;		//0x04
			REGDEF_NAME(adc, cr) CR;		//0x08
			union {
				REGDEF_NAME(adc, com_cfgr1) CFGR1;	//0x0C
				REGDEF_NAME(f_g_adc, cfgr1) F_G_CFGR1;	//0x0C
				REGDEF_NAME(h_adc, cfgr1) H_CFGR1;	//0x0C
			};
			REGDEF_NAME(adc, cfgr2) CFGR2;	//0x10
			union {
				REGDEF_NAME(f_g_adc, smpr) F_G_SMPR1;					//0x14 // Layout depnds on the chip
				REGDEF_NAME(h_adc, smpr) H_SMPR1;					//0x14 // Layout depnds on the chip
			};
			REGDEF_NAME(adc, smpr2) SMPR2;	//0x18 H503 only
			REGDEF_R(0x1C);
			REGDEF_NAME(adc, tr) AWD1TR;		//0x20
			REGDEF_NAME(adc, tr) AWD2TR;		//0x24
			uint32_t CHSELR_OR_TR3;				//0x28
			REGDEF_NAME(adc, tr) AWD3TR;		//0x2C
			REGDEF_NAME(adc, sqr1) SQR1;		//0x30
			REGDEF_NAME(adc, sqr2) SQR2;		//0x34
			REGDEF_NAME(adc, sqr3) SQR3;		//0x38
			REGDEF_NAME(adc, sqr4) SQR4;		//0x3C
			REG_S32(DR, 16);				//0x40
		} QEMU_PACKED;
		uint32_t raw[RI_END];
	} regs;

	qemu_irq irq;
	qemu_irq irq_read[STM32_COM_ADC_MAX_REG_CHANNELS]; // Set when the ADC wants to get a value from the channel.

	int adc_data[STM32_COM_ADC_MAX_REG_CHANNELS]; // Store the peripheral data received.
	QEMUTimer* next_eoc;

	uint8_t adc_sequence_position;

	int adc_type;

	COM_STRUCT_NAME(Adcc) *adcc;

	stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Adc);

#include "../stm32_registers/generated/stm32f030/ADC_reginfo.h"
#include "../stm32_registers/generated/stm32g070/ADC_reginfo.h"
#include "../stm32_registers/generated/stm32h503/ADC_reginfo.h"

static const stm32_periph_variant_t stm32_adc_variants[3] = {
 	{TYPE_STM32F030_ADC, stm32_f030_adc_reginfo},
 	{TYPE_STM32G070_ADC, stm32_g070_adc_reginfo},
	{TYPE_STM32H503_ADC, stm32_h503_adc_reginfo},
};

typedef struct COM_CLASS_NAME(Adc) {
	STM32PeripheralClass parent_class;
	stm32_reginfo_t var_reginfo[RI_END];
	int type_table;
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
    // We can't reset the data here because it might
    // clear the initial stuff sent by other device resets.
	// memset(&s->adc_data,0,STM32_COM_ADC_MAX_REG_CHANNELS*sizeof(int));
	if (s->next_eoc)
		timer_del(s->next_eoc);
}

// Technically these should all include half a nanosecond but
// to keep it as integer math we include the .5ns from the conversion time here.
static uint16_t f030_2x_smpr[] = { 2, 8, 14, 29, 42, 56, 72, 240 };
static uint16_t g070_2x_smpr[] = { 2, 4,  8, 13, 20, 40, 80, 161 };
static uint16_t h503_2x_smpr[] = { 3, 7,  13, 25, 48, 93, 248, 641 };
static uint8_t align_shifts[4] = { 4, 6, 8, 2};

static uint16_t adc_lookup_smpr(COM_STRUCT_NAME(Adc) *s, uint8_t value) {
	assert(value < 8);
	switch(s->adc_type)
	{
		case TYPE_G070:
			return g070_2x_smpr[value];
		case TYPE_F030:
			return f030_2x_smpr[value];
		default: // LCOV_EXCL_LINE
			g_assert_not_reached(); // LCOV_EXCL_LINE
	}
}

static uint8_t adc_h503_lookup_channel(COM_STRUCT_NAME(Adc) *s)
{
	switch (s->adc_sequence_position)
	{
		case 0 ... 3:
			return ((s->regs.SQR1.raw >> 6U) >> (s->adc_sequence_position * 6U)) & 0b11111U;
		case 4 ... 8:
			return (s->regs.SQR2.raw >> ((s->adc_sequence_position-4) * 6U)) & 0b11111U;
		case 9 ... 13:
			return (s->regs.SQR3.raw >> ((s->adc_sequence_position-9) * 6U)) & 0b11111U;
		case 14 ... 15:
			return (s->regs.SQR4.raw >> ((s->adc_sequence_position-14) * 6U)) & 0b11111U;
		default:
			g_assert_not_reached();
	}
}

static uint32_t stm32_adc_get_value(COM_STRUCT_NAME(Adc) *s)
{
	uint32_t internal_value = 0;
	bool align = false;
	switch (s->adc_type)
	{
		case TYPE_F030:
		case TYPE_G070:
			internal_value = s->adc_data[s->adc_sequence_position];
			align = s->regs.F_G_CFGR1.bits.ALIGN;
			break;
		case TYPE_H503:
			internal_value = s->adc_data[adc_h503_lookup_channel(s)];
			align = s->regs.H_CFGR1.bits.ALIGN;
			break;
		default:
			g_assert_not_reached();
	}
	// I'm not sure why this is yet - some sort of built in oversampling
	// that is enabled in non-DMA mode?
	// Mask: RES 0..3 == 12..6 bit mask, so shift right 2* RES
	internal_value >>= (s->regs.CFGR1.bits.RES<<1); // Raw value is max 12 bits, with up to 20 bits internal OVS for a final 16 bit value.

	if (!s->regs.CFGR1.bits.DMAEN) {
		if (s->regs.CFGR2.bits.ROVSE)
		{
			internal_value*= (1U << (s->regs.CFGR2.bits.OVSR + 1));
			internal_value >>= s->regs.CFGR2.bits.OVSS;
		}
		// I'm pretty sure here we just want to return internal_value.
	}
	s->regs.DR = internal_value & UINT16_MAX;
	//printf("ADC DR read for chan %u (%u)\n", s->adc_sequence_position, s->regs.defs.DR);
	if (align && !s->regs.CFGR2.bits.ROVSE) {
		return (s->regs.DR << align_shifts[s->regs.CFGR1.bits.RES]);
	} else {
		return s->regs.DR;
	}
}

// ADC data in from peripherals
static void stm32_adc_data_in(void *opaque, int n, int level){
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);
	s->adc_data[n] = level;
	// printf("ADC: Ch %d new data: %d\n",n, level);
}

static void stm32_adc_f_g_schedule_next(COM_STRUCT_NAME(Adc) *s) {
	if (!s->regs.CR.bits.ADEN || s->regs.CHSELR_OR_TR3 == 0 || !stm32_rcc_if_check_periph_clk(&s->parent))
	{
		return;
	}

	// Calculate the clock rate
	uint64_t clock = s->parent.clock_freq;

	if (s->adcc)
	{
		clock /= stm32_common_adcc_get_adcpre(s->adcc);
	}

	// #bits:
	uint32_t conv_cycles = (12U - (s->regs.CFGR1.bits.RES<<1U));
	bool rate_sel = (s->regs.F_G_SMPR1.bits.SMPSEL >> s->adc_sequence_position) & 1U;
	conv_cycles += (adc_lookup_smpr(s,
			rate_sel? s->regs.F_G_SMPR1.bits.SMP2 : s->regs.F_G_SMPR1.bits.SMP1
		));

	uint64_t delay_ns = (NANOSECONDS_PER_SECOND * conv_cycles) / clock;
	// printf("ADC conversion: %u cycles @ %"PRIu64" Hz (%lu nSec)\n", conv_cycles, clock, delay_ns);
	timer_mod_ns(s->next_eoc, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+delay_ns);

}

static void stm32_adc_h_schedule_next(COM_STRUCT_NAME(Adc) *s) {
	if (!s->regs.CR.bits.ADEN || !stm32_rcc_if_check_periph_clk(&s->parent))
	{
		return;
	}

	// Calculate the clock rate
	uint64_t clock = s->parent.clock_freq;

	if (s->adcc)
	{
		clock /= stm32_common_adcc_get_adcpre(s->adcc);
	}

	// #bits:
	uint32_t conv_cycles = (12U - (s->regs.CFGR1.bits.RES<<1U));
	conv_cycles += s->regs.H_SMPR1.bits.SMPPLUS;
	// Get conversion rate:
	uint8_t data_channel = adc_h503_lookup_channel(s);
	uint8_t smpr_index = 0;
	switch (data_channel)
	{
		case 0 ... 9:
			smpr_index = s->regs.H_SMPR1.raw >> (data_channel * 3U) & 0b111U;
			break;
		case 10 ... 18:
			smpr_index = s->regs.SMPR2.raw >> ((data_channel-10) * 3U) & 0b111U;
			break;
	}

	conv_cycles += h503_2x_smpr[smpr_index];
	uint64_t delay_ns = (NANOSECONDS_PER_SECOND * conv_cycles) / clock;
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

	CHECK_BOUNDS_R_V2(addr, RI_END, s->reginfo); // LCOV_EXCL_LINE

	uint32_t data = s->regs.raw[addr];

	switch (addr) {
	case RI_ISR ... RI_CHSELR:
		break;
	case RI_DR:
		if (s->regs.CR.bits.ADEN && s->regs.ISR.bits.EOC) {
			s->regs.ISR.bits.EOC ^= s->regs.ISR.bits.EOC;
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
	qemu_irq_raise(s->irq_read[s->adc_sequence_position]); // Toggle the data read request IRQ. The receiver can opt to send a new value (or do nothing)
}

static void stm32_adc_update_irqs(COM_STRUCT_NAME(Adc) *s, int level) {

	bool bChanged = level^s->regs.ISR.bits.EOC;
	s->regs.ISR.bits.EOC = level;

	if (s->regs.IER.bits.EOC && level && bChanged)
	{
		qemu_irq_raise(s->irq);
	}
}

static inline void stm32_f_g_adc_set_next_channel(COM_STRUCT_NAME(Adc) *s)
{
	uint32_t chanmask = s->regs.raw[RI_CHSELR] >> (s->adc_sequence_position + 1U);
	s->adc_sequence_position++;
	while (!(chanmask & 0x1))
	{
		s->adc_sequence_position++;
		chanmask >>= 1;
		if (s->adc_sequence_position > STM32_COM_ADC_MAX_REG_CHANNELS) // Looped, we went off the end.
		{
			s->adc_sequence_position = 0;
			chanmask = s->regs.raw[RI_CHSELR];
		}
	}
	// printf("Next ADC channel: %u\n", s->adc_sequence_position);
}

static inline void stm32_h_adc_set_next_channel(COM_STRUCT_NAME(Adc) *s)
{
	uint32_t len = s->regs.SQR1.bits.L;
	// Len is 0...n for 1...n+1 items in the sequence.
	s->adc_sequence_position++;
	if (s->adc_sequence_position > len)
	{
		s->adc_sequence_position = 0;
	}
	// printf("Next ADC channel: %u\n", s->adc_sequence_position);
}

static void stm32_adc_set_next_channel(COM_STRUCT_NAME(Adc) *s)
{
	switch (s->adc_type)
	{
		case TYPE_F030:
		case TYPE_G070:
			stm32_f_g_adc_set_next_channel(s);
			break;
		case TYPE_H503:
			stm32_h_adc_set_next_channel(s);
			break;
	}
}

static bool stm32_adc_channels_set(COM_STRUCT_NAME(Adc) *s)
{
	switch (s->adc_type)
	{
		case TYPE_F030:
		case TYPE_G070:
			return s->regs.CHSELR_OR_TR3 != 0;
		case TYPE_H503:
			return true; // H503 can't have zero items in the sequence.
		default:
			return false;
	}
}

static void stm32_adc_eoc_deadline(void *opaque) {

	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);
	if (!s->regs.CFGR1.bits.DMAEN) // This is probably wrong, this should be called regardless, but the old version of doing it after the convert was working...
	{
		stm32_adc_set_next_channel(s);
	}
	stm32_adc_convert(s);
	if (s->regs.IER.bits.EOC || s->adc_sequence_position == 0)
	{
		// Either end of cycle or end-of-sequence.
		stm32_adc_update_irqs(s, 1);
	}

	if (s->regs.CFGR1.bits.DMAEN)
	{
		s->regs.ISR.bits.EOC = 1;
		qemu_set_irq(s->parent.dmar[DMAR_P2M], s->mmio.addr + (4U*RI_DR));
		if (s->regs.CFGR1.bits.CONT && stm32_adc_channels_set(s))
		{
			stm32_adc_set_next_channel(s);
			switch (s->adc_type)
			{
				case TYPE_F030:
				case TYPE_G070:
					if (!s->regs.F_G_CFGR1.bits.WAIT) // don't schedule if in WAIT mode.
					{
						stm32_adc_f_g_schedule_next(s);
					}
					break;
				case TYPE_H503:
					stm32_adc_h_schedule_next(s);
					break;
			}
		}
	}
	else if (!s->regs.CFGR1.bits.CONT && s->regs.CFGR1.bits.EXTEN == 0)
	{
		if (s->regs.CFGR1.bits.DISCEN)
		{
			printf("FIXME - unimplemented DISCEN EOC mode\n");
		}
		else
		{
			s->regs.CR.bits.ADSTART = 0; // Stop the ADC, single conversion.
			s->regs.ISR.bits.EOC = 1;
		}
	}
}

static void stm32_adc_write(void *opaque, hwaddr addr,
					   uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Adc) *s = STM32COM_ADC(opaque);

	// printf("ADC_write %d : 0x%" HWADDR_PRIx ", Value: 0x%x\n",
    //         s->parent.periph, addr, (uint32_t)data);

	uint8_t offset = addr&0x3;
	addr>>=2; // Get index in array.
	CHECK_BOUNDS_W_V2(addr, data, RI_END); // LCOV_EXCL_LINE
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);
	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[addr], s->reginfo, addr);

	switch (addr) {
		case RI_CFGR1:
		case RI_CFGR2:
		case RI_IER:
		case RI_TR1:
		case RI_TR2:
		case RI_G070_TR3:
		case RI_SMPR:
		case RI_SQR1 ... RI_SQR4:
		case RI_CHSELR: // NOTE: shares address with H503 TR3
			s->regs.raw[addr] = data;
			break;
		case RI_CR:
		{
			REGDEF_NAME(adc, cr) new = {.raw = data };
			// Most new bits are RS, except VREGEN.
			s->regs.raw[addr] |= data;
			s->regs.CR.bits.ADVREGEN = new.bits.ADVREGEN;
			if (s->regs.CR.bits.ADEN && !s->regs.CR.bits.ADDIS)
			{
				s->regs.ISR.bits.ADRDY = 1;
			}
			if (s->regs.CR.bits.ADSTART)
			{
				switch (s->adc_type)
				{
					case TYPE_F030:
					case TYPE_G070:
						if (!s->regs.F_G_CFGR1.bits.WAIT) // don't schedule if in WAIT mode.
						{
							stm32_adc_f_g_schedule_next(s);
						}
						break;
					case TYPE_H503:
						stm32_adc_h_schedule_next(s);
						break;
				}
			}
			if (s->regs.CR.bits.ADCAL)
			{
				s->regs.CR.bits.ADCAL = 0; // No calibration required :)
			}
			if (s->regs.CR.bits.ADDIS)
			{
				s->regs.CR.bits.ADEN = 0;
				s->regs.ISR.bits.ADRDY = 0;
				s->regs.CR.bits.ADDIS = 0;
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
	CHECK_ALIGN(sizeof(s->regs),sizeof(s->regs.raw), "Raw array");
	// Check the bitfields. S32s should be fine because
	// the macro handles the padding math and problems are detected by the overall size change above
	CHECK_REG_u32(s->regs.ISR);
	CHECK_REG_u32(s->regs.IER);
	CHECK_REG_u32(s->regs.CR);
	CHECK_REG_u32(s->regs.CFGR1);
	CHECK_REG_u32(s->regs.CFGR2);
	CHECK_REG_u32(s->regs.AWD1TR);
	CHECK_REG_u32(s->regs.AWD2TR);
	CHECK_REG_u32(s->regs.AWD3TR);
	CHECK_REG_u32(s->regs.CHSELR_OR_TR3);
	CHECK_UNION(COM_STRUCT_NAME(Adc), regs.CHSELR_OR_TR3, regs.raw[RI_CHSELR]);


	s->next_eoc = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_adc_eoc_deadline, s);


	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

	qdev_init_gpio_out_named(DEVICE(obj), s->irq_read, "adc_read", STM32_COM_ADC_MAX_REG_CHANNELS);

	qdev_init_gpio_in_named(DEVICE(obj),stm32_adc_data_in, "adc_data_in", STM32_COM_ADC_MAX_REG_CHANNELS);

	STM32_MR_IO_INIT(&s->mmio, obj, &stm32_adc_ops, s, 512);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
	COM_CLASS_NAME(Adc) *k = STM32COM_ADC_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
	s->adc_type = k->type_table;
}

static void stm32_adc_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_adc_reset);
	dc->vmsd = &vmstate_stm32_adc;

	device_class_set_props(dc, stm32common_adc_properties);

	COM_CLASS_NAME(Adc) *k = STM32COM_ADC_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	if (data == stm32_f030_adc_reginfo)
	{
		k->type_table = TYPE_F030;
	}
	else if (data == stm32_g070_adc_reginfo)
	{
		k->type_table = TYPE_G070;
	} else if (data == stm32_h503_adc_reginfo)
	{
		k->type_table = TYPE_H503;
	}
	else
	{
		k->type_table = TYPE_UNKNOWN;
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
