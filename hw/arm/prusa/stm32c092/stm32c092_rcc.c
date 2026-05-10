/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 * (STM32C092x variants)
 *
 * Copyright 2026 by VintagePC <http://github.com/vintagepc>
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0360 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */


#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include <stdio.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "qemu/units.h"
// #define STATE_DEBUG_VAR rcc_dbg
#include "../utility/macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_rcc.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "../stm32_common/stm32_rcc_regdefs.h"
#include "../stm32f407/stm32_clktree.h"
#include "../stm32f407/stm32.h"
#include "../stm32_common/stm32_clk.h"

/* DEFINITIONS*/

/* See README for DEBUG details. */
//#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32F2XX_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define IS_RESET_VALUE(new_value, mask, reset_value) ((new_value & mask) == (mask & reset_value))

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

static const uint8_t AHB_PERIPHS[32] = {
    STM32_P_DMA1, 0, 0, 0, 0, 0, 0, 0,
    STM32_P_FSMC, 0, 0, 0, STM32_P_CRC, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB1_PERIPHS[32] = {
    STM32_P_TIM2, STM32_P_TIM3, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, STM32_P_CAN1, 0, STM32_P_SPI2, STM32_P_USBHS,
    STM32_P_CRS, STM32_P_USART2, STM32_P_USART3, STM32_P_USART4, 0, STM32_P_I2C1, 0, 0,
    0, 0, 0, STM32_P_DBG, STM32_P_PWR, 0, 0, 0
};

static const uint8_t APB2_PERIPHS[32] = {
    STM32_P_SYSCFG, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_USART1, STM32_P_TIM14,
    STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, STM32_P_ADC1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t GPIO_PERIPHS[32] = {
    STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, 0, STM32_P_GPIOF, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t (*PERIPH_BLOCKS[4])[32] = {
	&GPIO_PERIPHS, &AHB_PERIPHS, &APB1_PERIPHS, &APB2_PERIPHS
};

static const uint16_t AHBPRE_OPTS[16] = {
	[0b0000 ... 0b0111] = 1,
	[0b1000] = 2,
	[0b1001] = 4,
	[0b1010] = 8,
	[0b1011] = 16,
	[0b1100] = 64,
	[0b1101] = 128,
	[0b1110] = 256,
	[0b1111] = 512
};

static const uint8_t APBPRE_OPTS[8] = {
	[0b000 ... 0b011] = 1,
	[0b100] = 2,
	[0b101] = 4,
	[0b110] = 8,
	[0b111] = 16
};

#include "../stm32_registers/generated/stm32c092/RCC_index.h"
#include "../stm32_registers/generated/stm32c092/RCC_registers.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32C092_STRUCT_NAME(Rcc), STM32C092_RCC);

typedef struct STM32C092_STRUCT_NAME(Rcc) {
    /* Inherited -- MUST MATCH stm32_rcc.h. NO EXCEPTIONS. */
    STM32COMRccState parent;

    /* Additional clocks */
    Clk_t HSISYS,
	HSIKER,
    SYSCLK,
    HCLK, /* Output from AHB Prescaler */
	HCLK8,
	PCLK,
	RESERVED, /* Dummy clock for reserved non-contiguous values.*/
    TIMPCLK; /* timer clock */

	REGDEF_NAME(c092,rcc) regs;

	bool in_reset;

	qemu_irq* hclk_upd_irq;

} STM32C092_STRUCT_NAME(Rcc);


enum sw_src
{
	SW_HSISYS,
	SW_HSE,
	C071_USB48,
	SW_LSI,
	SW_LSE,
};

/* REGISTER IMPLEMENTATION */

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */

static void stm32_c092_rcc_update_irq(STM32C092_STRUCT_NAME(Rcc) *s)
{
	uint32_t value = s->regs.CIFR.raw & s->regs.CIER.raw;
	qemu_set_irq(s->parent.irq, value>0);
}

static void stm32_rcc_RCC_CR_write(STM32C092_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(c092_rcc,cr) new = { .raw = new_value };

    clktree_set_enabled(&s->parent.HSECLK, new.bits.HSEON);
	s->regs.CR.bits.HSERDY = s->regs.CR.bits.HSEON = new.bits.HSEON;
	s->regs.CIFR.bits.HSERDYF = new.bits.HSEON && s->regs.CIER.bits.HSERDYIE;

	clktree_set_scale(&s->HSISYS, 1, 1U<<new.bits.HSIDIV);

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.bits.HSEON && !init) &&
       (s->regs.CFGR.bits.SW == SW_HSISYS)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSICLK, new.bits.HSION);
	s->regs.CR.bits.HSIRDY = s->regs.CR.bits.HSION = new.bits.HSION;
	s->regs.CIFR.bits.HSIRDYF = new.bits.HSION && s->regs.CIER.bits.HSIRDYIE;

	clktree_set_enabled(&s->HSIKER, new.bits.HSIKERON);
	clktree_set_scale(&s->HSIKER, 1,new.bits.HSIKERDIV + 1U);
	s->regs.CR.bits.HSIKERON = new.bits.HSIKERON;

	clktree_set_scale(&s->SYSCLK, 1,new.bits.SYSDIV + 1U);

	stm32_c092_rcc_update_irq(s);

}

static void stm32_rcc_RCC_CFGR_write(STM32C092_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(c092_rcc,cfgr) new = { .raw = new_value };
	// SW
	if (new.bits.SW > 4 || new.bits.SW == C071_USB48)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid clock source for SYSCLK selected!");
	}
	else
	{
		clktree_set_selected_input(&s->SYSCLK, new.bits.SW);
		s->regs.CFGR.bits.SWS = s->regs.CFGR.bits.SW = new.bits.SW;
	}

	// HPRE:
	clktree_set_scale(&s->HCLK, 1, AHBPRE_OPTS[new.bits.HPRE]);
	s->regs.CFGR.bits.HPRE = new.bits.HPRE;

    /* PPRE */
    s->regs.CFGR.bits.PPRE = new.bits.PPRE;
	clktree_set_scale(&s->PCLK, 1, APBPRE_OPTS[new.bits.PPRE]);

	clktree_set_scale(&s->TIMPCLK, (APBPRE_OPTS[new.bits.PPRE] == 1U ? 1U : 2U), 1);
}

static void stm32_rcc_RCC_CCIPR_write(STM32C092_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(c092_rcc,ccipr) new = { .raw = new_value };

	clktree_set_selected_input(&s->parent.pclocks[STM32_P_USART1], new.bits.USART1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2C1], new.bits.I2C1SEL);
	// clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2S1], new.bits.I2S1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_CAN1], new.bits.FDCAN1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_ADC1], new.bits.ADCSEL);
	s->regs.CCIPR.raw = new.raw;
}

static void stm32_rcc_RCC_CSR1_write(STM32C092_STRUCT_NAME(Rcc) *s, uint8_t new_value, bool init)
{
	REGDEF_NAME(c092_rcc,csr1) val = {.raw = new_value};
    clktree_set_enabled(&s->parent.LSECLK, val.bits.LSEON);
	s->regs.CSR1.bits.LSCOEN = s->regs.CSR1.bits.LSERDY = val.bits.LSEON;


	if (s->regs.CSR1.bits.RTCSEL)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Cannot change RTC clock source after it has been set!");
	}
	else if (val.bits.RTCSEL == 0)
	{
		clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], 0);
	}
	else
	{
		clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], val.bits.RTCEN);
		clktree_set_selected_input(&s->parent.pclocks[STM32_P_RTC], val.bits.RTCSEL - 1U );
	}

}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR2_write(STM32C092_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	REGDEF_NAME(c092_rcc,csr2) val = { .raw = new_value };
    clktree_set_enabled(&s->parent.LSICLK, val.bits.LSION);
	val.bits.LSIRDY = val.bits.LSION;
	s->regs.raw[RI_CSR2] = val.raw;
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    STM32C092_STRUCT_NAME(Rcc) *s = STM32C092_RCC(opaque);
	uint32_t offset = addr & 0x3U;
	uint32_t index = addr >> 2U;
	CHECK_BOUNDS_R(index, RI_END, stm32_c092_rcc_reginfo, "RCC");

	uint32_t value = s->regs.raw[index];

	ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b111);
    return value;
}



static void stm32_rcc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
	STM32C092_STRUCT_NAME(Rcc) *s = STM32C092_RCC(opaque);
	uint32_t offset = addr & 0x3U;
	addr >>= 2U;

	CHECK_BOUNDS_W_V2(addr, value, RI_END);

	CHECK_UNIMP_RESVD_V2(value, s->regs.raw[addr], stm32_c092_rcc_reginfo, addr);

    ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], value, size, offset, 0b111);

    switch (addr)
    {
		case RI_CR:
            stm32_rcc_RCC_CR_write(s, value, s->in_reset);
            break;
		case RI_CIER:
			s->regs.CIER.raw = value;
			stm32_c092_rcc_update_irq(s);
			break;
		case RI_CICR:
			s->regs.CIFR.raw &= ~value;
			stm32_c092_rcc_update_irq(s);
			break;
        case RI_CFGR:
            stm32_rcc_RCC_CFGR_write(s, value, s->in_reset);
            break;
        case RI_CCIPR:
            stm32_rcc_RCC_CCIPR_write(s, value, s->in_reset);
            break;
		case RI_IOPRSTR ... RI_APBRSTR2:
			stm32_common_rcc_reset_write(&s->parent,value, PERIPH_BLOCKS[addr - RI_IOPRSTR]);
			s->regs.raw[addr] = value;
			break;
		case RI_IOPENR ... RI_APBENR2:
            stm32_common_rcc_enable_write(&s->parent, value, PERIPH_BLOCKS[addr - RI_IOPENR]);
			s->regs.raw[addr] = value;
            break;
		case RI_IOPSMENR ... RI_APBSMENR2:
			//printf("STOP MODE rcc write not implemetned!\n");
			s->regs.raw[addr] = value;
			break;
        case RI_CSR1:
            stm32_rcc_RCC_CSR1_write(s, value, s->in_reset);
            break;
        case RI_CSR2:
            stm32_rcc_RCC_CSR2_write(s, value, s->in_reset);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    STM32C092_STRUCT_NAME(Rcc) *s = STM32C092_RCC(dev);

	s->in_reset = true;
	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (stm32_c092_rcc_reginfo[i].not_reserved)
		{
			stm32_rcc_write(dev, i<<2U, stm32_c092_rcc_reginfo[i].reset_val, 4U);
		}
	}
	s->in_reset = false;
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    STM32C092_STRUCT_NAME(Rcc) *s = (STM32C092_STRUCT_NAME(Rcc) *)opaque;

    uint32_t hclk_freq = clktree_get_output_freq(&s->HCLK);

	clock_set_hz(s->parent.CPUCLOCK, hclk_freq);
	printf("# CPUCLOCK set to %u Hz\n",hclk_freq);
	clock_propagate(s->parent.CPUCLOCK);
	clock_set_hz(s->parent.REFCLK, hclk_freq/8);
	clock_propagate(s->parent.REFCLK);
	printf("# Systick frequency (REFCLK) set to %u Hz\n", hclk_freq/8);

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
#endif
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_realize(DeviceState *dev, Error **errp)
{
    STM32C092_STRUCT_NAME(Rcc) *s = STM32C092_RCC(dev);

	s->parent.realize_func(dev, errp);

    int i;
   	s->hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_P_COUNT; i++) {
        s->parent.pclocks[i].is_initialized = false;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */

	 clktree_create_src_clk(&s->RESERVED, "RESERVED", 0, false);

	clktree_create_clk(&s->HSISYS, "HSISYS", 1, 1, true, s->parent.hsi_freq, 0, &s->parent.HSICLK, NULL);
	clktree_create_clk(&s->HSIKER, "HSIKER", 1, 1, true, s->parent.hsi_freq, 0, &s->parent.HSICLK, NULL);

	// TODO - determine max sysclk freq.
    clktree_create_clk(&s->SYSCLK, "SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   &s->HSISYS, &s->parent.HSECLK, &s->RESERVED, &s->parent.LSICLK, &s->parent.LSECLK, NULL);

    clktree_create_clk(&s->HCLK, "HCLK", 1, 1, true, 64000000, 0, &s->SYSCLK, NULL);
    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);
    clktree_create_clk(&s->HCLK8, "HCLK8", 1, 8, true, 8000000, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PCLK, "PCLK", 1, 1, true, 64000000, 0, &s->HCLK, NULL);

	clktree_create_clk(&s->TIMPCLK, "TIMPCLK", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);

    /* AHB Peripheral clocks */
    INIT_PCLK(GPIOA, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOB, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOD, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOF, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);


// verified
    INIT_PCLK(CRC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(FSMC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(EXTI, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(RCC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMAMUX, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMA1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMA2, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    INIT_PCLK(USART1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->HSIKER, &s->parent.LSECLK, NULL);
    INIT_PCLK(USART2, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);
    INIT_PCLK(USART3, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);

    INIT_PCLK(CAN1,  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->HSIKER, &s->parent.HSECLK, NULL);

    INIT_PCLK(I2C1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->HSIKER, NULL);


    INIT_PCLK(ADC1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->SYSCLK, &s->RESERVED, &s->HSIKER, NULL);

	// Everything else is APB

    INIT_PCLK(SYSCFG, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(USART4, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(USART5, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(USART6, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(TIM1,  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM2,  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM3,  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM14, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
	INIT_PCLK(TIM15, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM16, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM17, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);

// vf'd end



    INIT_PCLK(IWDG, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.LSICLK, NULL);
    INIT_PCLK(PWR, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->SYSCLK, NULL);
    INIT_PCLK(RTC, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.LSICLK, &s->parent.LSECLK, &s->parent.HSECLK, NULL);

}



static void stm32_rcc_init(Object *obj)
{

    STM32C092_STRUCT_NAME(Rcc) *s = STM32C092_RCC(obj);

    STM32_MR_IO_INIT(&s->parent.iomem, obj, &stm32_rcc_ops, s, 1U * KiB);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->parent.iomem);
}

static const VMStateDescription vmstate_stm32c092_rcc = {
    .name = TYPE_STM32C092_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(parent.pclocks, STM32C092_STRUCT_NAME(Rcc), STM32_P_COUNT, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(SYSCLK,STM32C092_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HCLK,STM32C092_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK,STM32C092_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
		VMSTATE_UINT32_ARRAY(regs.raw, STM32C092_STRUCT_NAME(Rcc), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_rcc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_rcc_reset);
    dc->realize = stm32_rcc_realize;
    dc->vmsd = &vmstate_stm32c092_rcc;
}

static TypeInfo stm32_rcc_info = {
    .name  = TYPE_STM32C092_RCC,
    .parent = TYPE_STM32COM_RCC,
    .instance_size  = sizeof(STM32C092_STRUCT_NAME(Rcc)),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init,
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
