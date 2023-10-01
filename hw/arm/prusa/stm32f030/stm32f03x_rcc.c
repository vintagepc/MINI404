/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 * (STM32F030x variants)
 *
 * Copyright 2021 by VintagePC <http://github.com/vintagepc>
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
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include <stdio.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/units.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "hw/arm/armv7m.h"
#include "../stm32f407/stm32_clktree.h"
#include "../stm32_common/stm32_clk.h"
#include "../stm32_common/stm32_rcc.h"
#include "../stm32_common/stm32_rcc_regdefs.h"

enum reg_index {
	RI_CR,
	RI_CFGR,
	RI_CIR,
	RI_APB2RSTR,
	RI_APB1RSTR,
	RI_AHBENR,
	RI_APB2ENR,
	RI_APB1ENR,
	RI_BDCR,
	RI_CSR,
	RI_AHBRSTR,
	RI_CFGR2,
	RI_CFGR3,
	RI_CR2,
	RI_MAX
};

OBJECT_DECLARE_SIMPLE_TYPE(STM32F030_STRUCT_NAME(Rcc), STM32F030_RCC);

REGDEF_BLOCK_BEGIN()
		uint32_t SW :2;
		uint32_t SWS :2;
		uint32_t HPRE :4;
		uint32_t PPRE :3;
		REG_R(3);
		REG_B32(ADCPRE);
		REG_RB();
		REG_B32(PLLSRC);
		REG_B32(PLLXTPRE);
		uint32_t PLLMUL :4;
		REG_R(2);
		uint32_t MCO :4;
		uint32_t MCOPRE :3;
		REG_B32(PLLNODIV);
REGDEF_BLOCK_END(rcc, cfgr);

REGDEF_BLOCK_BEGIN()
	REG_RB();
	REG_B32(TIM3);
	REG_R(2);
	REG_B32(TIM6);
	REG_B32(TIM7);
	REG_R(2);
	REG_B32(TIM14);
	REG_R(2);
	REG_B32(WWDG);
	REG_R(2);
	REG_B32(SPI2);
	REG_R(2);
	REG_B32(USART2);
	REG_B32(USART3);
	REG_B32(USART4);
	REG_B32(USART5);
	REG_B32(I2C1);
	REG_B32(I2C2);
	REG_B32(USB);
	REG_R(3);
	REG_B32(CRS);
	REG_B32(PWR);
	REG_R(3);
REGDEF_BLOCK_END(rcc, apb1);

REGDEF_BLOCK_BEGIN()
	REG_B32(SYSCFG);
	REG_R(4);
	REG_B32(USART6);
	REG_R(3);
	REG_B32(ADC);
	REG_RB();
	REG_B32(TIM1);
	REG_B32(SPI1);
	REG_RB();
	REG_B32(USART1);
	REG_RB();
	REG_B32(TIM15);
	REG_B32(TIM16);
	REG_B32(TIM17);
	REG_R(3);
	REG_B32(DBGMCU);
	REG_R(9);
REGDEF_BLOCK_END(rcc, apb2);

REGDEF_BLOCK_BEGIN()
	REG_B32(DMAEN);
	REG_RB();
	REG_B32(SRAMEN);
	REG_RB();
	REG_B32(FLITFEN);
	REG_RB();
	REG_B32(CRCEN);
	REG_R(10);
	REG_B32(IOPAEN);
	REG_B32(IOPBEN);
	REG_B32(IOPCEN);
	REG_RB();
	REG_B32(IOPEEN);
	REG_R(9);
REGDEF_BLOCK_END(rcc, ahbenr);

REGDEF_BLOCK_BEGIN()
	REG_R(17);
	REG_B32(IOPARST);
	REG_B32(IOPBRST);
	REG_B32(IOPCRST);
	REG_B32(IOPDRST);
	REG_RB();
	REG_B32(IOPFRST);
	REG_R(9);
REGDEF_BLOCK_END(rcc, ahbrst);

REGDEF_BLOCK_BEGIN()
	REG_B32(LSION);
	REG_B32(LSIRDY);
	REG_R(22);
	REG_B32(RMVF);
	REG_B32(OBLRSTF);
	REG_B32(PINRSTF);
	REG_B32(PORRSTF);
	REG_B32(SFTRSTF);
	REG_B32(IWDGRSTF);
	REG_B32(WWDGRSTF);
	REG_B32(LPWRSTF);
REGDEF_BLOCK_END(rcc, csr);

REGDEF_BLOCK_BEGIN()
		REG_B32(HSI14ON);
		REG_B32(HSI14RDY);
		REG_B32(HSI14DIS);
		REG_K32(HSITRIM,5);
		REG_K32(HSICAL ,8);
		REG_R(16);
REGDEF_BLOCK_END(rcc,cr2);

REGDEF_BLOCK_BEGIN()
		REG_K32(USART1SW,2);
		REG_R(2);
		REG_B32(I2C1SW);
		REG_R(2);
		REG_B32(USBSW);
		REG_B32(ADCSW);
		REG_R(23);
REGDEF_BLOCK_END(rcc,cfgr3);

typedef struct STM32F030_STRUCT_NAME(Rcc) {
    /* Inherited */
    STM32COMRccState parent;

    /* Additional clocks */
    Clk_t HSI14,
    SYSCLK,

    PLLM, /* Applies "M" division and "N" multiplication factors for PLL */
	PLLPREDIV, // handles the HSE prediv for PLL
    PLLCLK,
    PLL48CLK,

	TIMCLK, // Timer prescaler

    HCLK, /* Output from AHB Prescaler */
    PCLK1; /* Output from APB Prescaler */

	union {
		struct {
			REGDEF_NAME(rcc_com,cr) CR;
			REGDEF_NAME(rcc,cfgr) CFGR;
			uint32_t CIR;
			REGDEF_NAME(rcc, apb2) APB2RSTR;
			REGDEF_NAME(rcc, apb1) APB1RSTR;
			REGDEF_NAME(rcc, ahbenr) AHBENR;
			REGDEF_NAME(rcc, apb2) APB2ENR;
			REGDEF_NAME(rcc, apb1) APB1ENR;
			REGDEF_NAME(rcc_com,bdcr) BDCR;
			uint32_t CSR;
			REGDEF_NAME(rcc, ahbrst) AHBRSTR;
			REG_S32(PREDIV,4) CFGR2;
			REGDEF_NAME(rcc,cfgr3) CFGR3;
			REGDEF_NAME(rcc,cr2) CR2;

		} QEMU_PACKED;
		uint32_t raw[RI_MAX];
	} regs;

	qemu_irq* hclk_upd_irq;

} STM32F030_STRUCT_NAME(Rcc);


#define HSI14_FREQ 14000000
#define AP_MAX_FREQ 48000000

static const uint8_t AHB1RST_PERIPHS[32] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, 0, STM32_P_GPIOF, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t AHB1ENA_PERIPHS[32] = {
    STM32_P_DMA1, 0, 0/*sram*/, 0, STM32_P_FSMC, 0, STM32_P_CRC, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, STM32_P_UART4, STM32_P_GPIOF, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB1_PERIPHS[32] = {
    0, STM32_P_TIM3, 0, 0, STM32_P_TIM6, STM32_P_TIM7, 0, 0,
    STM32_P_TIM14, 0, 0, STM32_P_WWDG, 0, 0, STM32_P_SPI2, 0,
    0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, STM32_P_UART5, STM32_P_I2C1, STM32_P_I2C2, STM32_P_USBHS,
    0, 0, 0, STM32_P_CRYP, STM32_P_PWR, 0, 0, 0
};

static const uint8_t APB2_PERIPHS[32] = {
    STM32_P_SYSCFG, 0, 0, 0, 0, STM32_P_UART6, 0, 0,
    0, STM32_P_ADC1, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_UART1, 0,
    STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

static const uint8_t (*RESET_BLOCKS[RI_MAX])[32] = {
	[RI_AHBRSTR] = &AHB1RST_PERIPHS,
	[RI_APB1RSTR] = &APB1_PERIPHS,
	[RI_APB2RSTR] = &APB2_PERIPHS,
};


static const uint8_t (*ENA_BLOCKS[RI_MAX])[32] = {
	[RI_AHBENR] = &AHB1ENA_PERIPHS,
	[RI_APB1ENR] = &APB1_PERIPHS,
	[RI_APB2ENR] = &APB2_PERIPHS,
};

static const stm32_reginfo_t stm32f030_rcc_reginfo[RI_MAX] =
{
	[RI_CR] = {.mask = 0x030FFFFB, .reset_val = 0x83},
	[RI_CFGR] = {.mask = 0xFF3F47FF, .unimp_mask = 0xFF000000},
	[RI_CIR] = {.mask = 0xBF3FBF, .unimp_mask = UINT32_MAX},
	[RI_APB2RSTR] = {.mask = 0x475A21, .unimp_mask = 0x400000},
	[RI_APB1RSTR] = {.mask = 0x18FE4932 },
	[RI_AHBENR] = {.mask = 0x3E0055, .reset_val = 0x14},
	[RI_APB2ENR] = { .mask = 0x475A41},
	[RI_APB1ENR] = {.mask = 0x18FE4932},
	[RI_BDCR] = {.mask = 0x1831F},
	[RI_CSR] = {.mask = 0xFF000003, .unimp_mask = 0xFF000000},
	[RI_AHBRSTR] = {.mask = 0x5E0000 },
	[RI_CFGR2] = {.mask = 0xF, .unimp_mask = 0xF },
	[RI_CFGR3] = {.mask = 0x193},
	[RI_CR2] = {.mask = UINT16_MAX, .reset_val = 0x80, .unimp_mask = UINT32_MAX},
};

enum SW
{
	SW_HSI,
	SW_HSE,
	SW_PLL,
};

/* REGISTER IMPLEMENTATION */

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(STM32F030_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc_com,cr) new = { .raw = new_value };

    if((clktree_is_enabled(&s->PLLCLK) && !new.PLLON) &&
       s->regs.CFGR.SW == SW_PLL) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(&s->PLLCLK, new.PLLON);

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.HSEON) &&
       (s->regs.CFGR.SW == SW_HSE || s->regs.CFGR.SW == SW_PLL)
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSECLK, new.HSEON);

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.HSEON) &&
       (s->regs.CFGR.SW == SW_HSI || s->regs.CFGR.SW == SW_PLL)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSICLK, new.HSION);
}

static void stm32_rcc_RCC_CFGR_write(STM32F030_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc,cfgr) new = { .raw = new_value };

    /* PPRE */
    s->regs.CFGR.PPRE = new.PPRE;
	uint8_t timer_scale = 1;
    if(s->regs.CFGR.PPRE < 4) {
        clktree_set_scale(&s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(&s->PCLK1, 1, 2 * (s->regs.CFGR.PPRE - 3));
		timer_scale = 2;
    }
	// Per datasheet, timers run at 2x if the divisor is >1
	clktree_set_scale(&s->TIMCLK, timer_scale, 1);


    /* HPRE */
    s->regs.CFGR.HPRE = new.HPRE;
    if(s->regs.CFGR.HPRE < 8) {
        clktree_set_scale(&s->HCLK, 1, 1);
    } else {
        clktree_set_scale(&s->HCLK, 1, 2 * (s->regs.CFGR.HPRE - 7));
    }

    /* SW */
    s->regs.CFGR.SW = new.SW;
    s->regs.CFGR.SWS = new.SW;
    switch(new.SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(&s->SYSCLK, new.SW);
			clock_set_hz(s->parent.CPUCLOCK, clktree_get_output_freq(&s->SYSCLK));
			clock_propagate(s->parent.CPUCLOCK);
            break;
        default:
            printf("Invalid input selected for SYSCLK");
            break;
    }

	s->regs.CFGR.PLLMUL = new.PLLMUL;
	uint8_t new_mul = new.PLLMUL+2U;
	if (new_mul>16) new_mul = 16;
	uint8_t new_div = (new.PLLSRC ? s->regs.CFGR2.PREDIV + 1U : 2U);
	clktree_set_scale(&s->PLLCLK, new_mul, new_div);
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
	STM32F030_STRUCT_NAME(Rcc) *s = STM32F030_RCC(opaque);
	uint32_t index = offset >> 2U;
	CHECK_BOUNDS_R(index, RI_MAX, stm32f030_rcc_reginfo, "F030 RCC");
	uint32_t data = 0;
    switch (index) {
        case RI_CR:
		{
			REGDEF_NAME(rcc_com,cr) r = { .raw = 0 };
			r.PLLON = r.PLLRDY = clktree_is_enabled(&s->PLLCLK);
			r.HSEON = r.HSERDY = clktree_is_enabled(&s->parent.HSECLK);
			r.HSION = r.HSIRDY = clktree_is_enabled(&s->parent.HSICLK);
		    data = r.raw;
			break;
		}
        case RI_CFGR:
			data = s->regs.CFGR.raw;
			break;
        case RI_CIR:
            data = s->regs.CIR;
			break;
        case RI_AHBENR:
            data = s->regs.AHBENR.raw;
			break;
        case RI_APB1ENR:
            data = s->regs.APB1ENR.raw;
			break;
        case RI_APB2ENR:
            data = s->regs.APB2ENR.raw;
			break;
        case RI_BDCR:
		{
			REGDEF_NAME(rcc_com,bdcr) retval = {
				.LSEON = clktree_is_enabled(&s->parent.LSECLK),
				.LSERDY = clktree_is_enabled(&s->parent.LSECLK) | true,
			};
			data = retval.raw;
			break;
		}
        case RI_CSR:
        {
			REGDEF_NAME(rcc,csr) retval = {
				.LSION = clktree_is_enabled(&s->parent.LSICLK),
				.LSIRDY = clktree_is_enabled(&s->parent.LSICLK) | true,
			};
    		data = retval.raw;
			break;
		}
		case RI_CR2:
		{
			REGDEF_NAME(rcc,cr2) r = { .raw = 0 };
			r.HSI14ON = r.HSI14RDY = clktree_is_enabled(&s->HSI14);
			data = r.raw;
			break;
		}
		case RI_CFGR3:
			data = s->regs.CFGR3.raw;
			break;
    }
	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, offset&0x3, 0b100);
	return data;
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
   	STM32F030_STRUCT_NAME(Rcc) *s = STM32F030_RCC(opaque);
	uint32_t index = offset>>2U;
	CHECK_BOUNDS_W(index, value, RI_MAX, stm32f030_rcc_reginfo, "F030 RCC");
	ADJUST_FOR_OFFSET_AND_SIZE_W(stm32_rcc_read(opaque, index<<2U,4), value, size, offset&0x3, 0b100);
	CHECK_UNIMP_RESVD(value, stm32f030_rcc_reginfo, index);
    switch(index) {
        case RI_CR:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RI_CFGR:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RI_APB1RSTR:
        case RI_APB2RSTR:
        case RI_AHBRSTR:
            stm32_common_rcc_reset_write(&s->parent, value, RESET_BLOCKS[index]);
			s->regs.raw[index] = value;
            break;
        case RI_APB1ENR:
        case RI_APB2ENR:
        case RI_AHBENR:
            stm32_common_rcc_enable_write(&s->parent, value, ENA_BLOCKS[index]);
			s->regs.raw[index] = value;
            break;
        case RI_BDCR:
		{
			REGDEF_NAME(rcc_com,bdcr) val = {.raw = value};
			clktree_set_enabled(&s->parent.LSECLK, val.LSEON);
			clktree_set_selected_input(&s->parent.pclocks[STM32_P_RTC], val.RTC_SEL - 1U);
			clktree_set_scale(&s->parent.pclocks[STM32_P_RTC], 1, val.RTC_SEL == 0b11 ? 32 : 1);
			clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], val.RTCEN);
			s->regs.raw[index] = value;
		}
		break;
        case RI_CSR:
			{
            	REGDEF_NAME(rcc,csr) val = { .raw = value };
    			clktree_set_enabled(&s->parent.LSICLK, val.LSION);
			}
			break;
		case RI_CFGR2:
			s->regs.raw[index] = value;
			break;
		case RI_CFGR3:
			{
				REGDEF_NAME(rcc, cfgr3) r = {.raw = value};
				clktree_set_enabled(&s->parent.pclocks[STM32_P_USBHS], r.USBSW);
				clktree_set_enabled(&s->parent.pclocks[STM32_P_I2C1], r.I2C1SW);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_UART1], r.USART1SW);
			}
			break;
        default:
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
    STM32F030_STRUCT_NAME(Rcc) *s = STM32F030_RCC(dev);

    for (int i=0; i<RI_MAX; i++)
		stm32_rcc_write(s, i<<2U, stm32f030_rcc_reginfo[i].reset_val,4);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    STM32F030_STRUCT_NAME(Rcc) *s = (STM32F030_STRUCT_NAME(Rcc) *)opaque;

    uint32_t hclk_freq = clktree_get_output_freq(&s->HCLK);

	clock_set_hz(s->parent.REFCLK, hclk_freq);
	clock_propagate(s->parent.REFCLK);
}



/* DEVICE INITIALIZATION */


/* Set up the clock tree */
static void stm32_rcc_realize(DeviceState *dev, Error **errp)
{
    STM32F030_STRUCT_NAME(Rcc) *s = STM32F030_RCC(dev);
	// Invoke parent func to set up main clocks
	s->parent.realize_func(dev, errp);

   	s->hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
	clktree_create_src_clk(&s->HSI14, "HSI14", HSI14_FREQ, true); // I believe this is enabled by default unless turned off by the ADC...
	clktree_create_clk(&s->PLLPREDIV, "PLLPREDIV", 1, 1, true, AP_MAX_FREQ, 0, &s->parent.HSECLK, NULL);
    clktree_create_clk(&s->PLLCLK, "PLLCLK", 1, 1, false, 48000000, 0, &s->parent.HSICLK, &s->PLLPREDIV, NULL);

    clktree_create_clk(&s->SYSCLK, "SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   &s->parent.HSICLK, &s->parent.HSECLK, &s->PLLCLK, NULL);
    clktree_create_clk(&s->HCLK, "HCLK", 1, 1, true, AP_MAX_FREQ, 0, &s->SYSCLK, NULL);
    clktree_create_clk(&s->PCLK1, "PCLK1", 1, 1, true, AP_MAX_FREQ, 0, &s->HCLK, NULL);

    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);

	clktree_create_clk(&s->TIMCLK, "TIMCLK", 1, 1, true, AP_MAX_FREQ, 0, &s->PCLK1, NULL);

    /* Peripheral clocks - special case */
    INIT_PCLK_NSM(FSMC, 0, &s->parent.HSICLK, NULL);
    INIT_PCLK_NSM(I2C1, 0, &s->parent.HSICLK, &s->SYSCLK, NULL);

    INIT_PCLK_NSM(TIM1,  0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM3,  0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM6,  0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM7,  0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM14, 0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM15, 0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM16, 0, &s->TIMCLK, NULL);
    INIT_PCLK_NSM(TIM17, 0, &s->TIMCLK, NULL);

    INIT_PCLK(ADC1, 1, 1, false, 14000000, 0, &s->HSI14, &s->PCLK1, NULL);

	INIT_PCLK_NSM(UART1, 0, &s->PCLK1, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);

	INIT_PCLK_NSM(RTC, 0, &s->parent.HSECLK, &s->parent.LSECLK, &s->parent.LSICLK, NULL);

    INIT_PCLK(IWDG, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.LSICLK, NULL);

	// everything else - AHB:
    INIT_PCLK_NSM(GPIOA, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOB, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOC, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOD, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOE, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOF, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(CRC, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(DMA1, 0, &s->HCLK, NULL);

	// APB periphs:
    INIT_PCLK_NSM(SPI1, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(SPI2, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(EXTI, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(SYSCFG, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART2, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART3, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART4, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART5, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART6, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(PWR, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(USBHS, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(I2C2, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(WWDG, 0, &s->PCLK1, NULL);

}



static void stm32_rcc_init(Object *obj)
{

    STM32F030_STRUCT_NAME(Rcc) *s = STM32F030_RCC(obj);

    STM32_MR_IO_INIT(&s->parent.iomem, obj, &stm32_rcc_ops, s, 1U * KiB);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->parent.iomem);
}

static const VMStateDescription vmstate_stm32f2xx_rcc = {
    .name = TYPE_STM32F030_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(HSI14,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(SYSCLK,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLM,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLPREDIV,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLCLK,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLL48CLK,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HCLK,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK1,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(TIMCLK,STM32F030_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
		VMSTATE_UINT32_ARRAY(regs.raw, STM32F030_STRUCT_NAME(Rcc), RI_MAX),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_rcc_reset;
    dc->realize = stm32_rcc_realize;
	dc->vmsd = &vmstate_stm32f2xx_rcc;
}

static TypeInfo stm32_rcc_info = {
    .name  = TYPE_STM32F030_RCC,
    .parent = TYPE_STM32COM_RCC,
    .instance_size  = sizeof(STM32F030_STRUCT_NAME(Rcc)),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init,
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
