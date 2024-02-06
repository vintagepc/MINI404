/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 * (STM32G070x variants)
 *
 * Copyright 2022 by VintagePC <http://github.com/vintagepc>
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
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include <stdio.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/units.h"
// #define STATE_DEBUG_VAR rcc_dbg
#include "../stm32_common/stm32_types.h"
#include "../utility/macros.h"
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

#define WARN_UNIMPLEMENTED(new_value, mask, reset_value) \
    if (!IS_RESET_VALUE(new_value, mask, reset_value)) { \
        stm32_unimp("Not implemented: RCC " #mask ". Masked value: 0x%08x\n", (new_value & mask)); \
    }

#define WARN_UNIMPLEMENTED_REG(offset) \
        stm32_unimp("STM32f2xx_rcc: unimplemented register: 0x%x", (int)offset)

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

static const uint8_t AHB_PERIPHS[32] = {
    STM32_P_DMA1, STM32_P_DMA2, 0, 0, 0, 0, 0, 0,
    STM32_P_FSMC, 0, 0, 0, STM32_P_CRC, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB1_PERIPHS[32] = {
    0, STM32_P_TIM3, STM32_P_TIM4, 0, STM32_P_TIM6, STM32_P_TIM7, 0, 0,
    STM32_P_UART5, STM32_P_UART6, 0, 0, 0, STM32_P_USBHS, STM32_P_SPI2, STM32_P_SPI3,
    0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, 0, STM32_P_I2C1, STM32_P_I2C2, STM32_P_I2C3,
    0, 0, 0, 0, STM32_P_PWR, 0, 0, 0
};

static const uint8_t APB2_PERIPHS[32] = {
    STM32_P_SYSCFG, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_UART1, STM32_P_TIM14,
    STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, STM32_P_ADC1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t GPIO_PERIPHS[32] = {
    STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, STM32_P_GPIOE, STM32_P_GPIOF, 0, 0,
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

enum reg_index {
	RI_CR,
	RI_ICSCR,
	RI_CFGR,
	RI_PLLCFGR,
	RI_RES1,
	RI_RES2,
	RI_CIER,
	RI_CIFR,
	RI_CICR,
	RI_IOPRSTR,
	RI_AHBRSTR,
	RI_APB1RSTR,
	RI_APB2RSTR,
	RI_IOPENR,
	RI_AHBENR,
	RI_ABP1ENR,
	RI_APB2ENR,
	RI_IOPSMENR,
	RI_AHBSMENR,
	RI_ABP1SMENR,
	RI_APB2SMENR,
	RI_CCIPR,
	RI_CCIPR2,
	RI_BDCR,
	RI_CSR,
	RI_END
};

OBJECT_DECLARE_SIMPLE_TYPE(STM32G070_STRUCT_NAME(Rcc), STM32G070_RCC);

REGDEF_BLOCK_BEGIN()
		REG_R(8);
		REG_B32(HSION);
		REG_B32(HSIKERON);
		REG_B32(HSIRDY);
		REG_K32(HSIDIV,3);
		REG_R(2);
		REG_B32(HSEON);
		REG_B32(HSERDY);
		REG_B32(HSEBYP);
		REG_B32(CSSON);
		REG_R(4);
		REG_B32(PLLON);
		REG_B32(PLLRDY);
		REG_R(6);
REGDEF_BLOCK_END(rcc,cr);

REGDEF_BLOCK_BEGIN()
	REG_K32(HSICAL,8);
	REG_K32(HSITRIM,7);
	REG_R(17);
REGDEF_BLOCK_END(rcc,icscr);

REGDEF_BLOCK_BEGIN()
		REG_K32(SW,3);
		REG_K32(SWS,3);
		REG_R(2);
		REG_K32(HPRE,4);
		REG_K32(PPRE,3);
		REG_RB();
		REG_K32(MCO2SEL,4);
		REG_K32(MCO2PRE,4);
		REG_K32(MCOSEL,4);
		REG_K32(MCOPRE,4);
REGDEF_BLOCK_END(rcc, cfgr);

REGDEF_BLOCK_BEGIN()
		REG_K32(PLLSRC,2);
		REG_R(2);
		REG_K32(PLLM,3);
		REG_RB();
		REG_K32(PLLN,7);
		REG_RB();
		REG_B32(PLLPEN);
		REG_K32(PLLP,5);
		REG_R(2);
		REG_B32(PLLQEN);
		REG_K32(PLLQ,3);
		REG_B32(PLLREN);
		REG_K32(PLLR,3);
REGDEF_BLOCK_END(rcc, pllcfgr);

REGDEF_BLOCK_BEGIN()
	REG_B32(LSIRDY);
	REG_B32(LSERDY);
	REG_RB();
	REG_B32(HSIRDY);
	REG_B32(HSERDY);
	REG_B32(PLLRDY);
	REG_R(2);
	REG_B32(CSS);
	REG_B32(LSECSS);
	REG_R(22);
REGDEF_BLOCK_END(rcc, inter);


REGDEF_BLOCK_BEGIN()
	REG_B32(GPIOA);
	REG_B32(GPIOB);
	REG_B32(GPIOC);
	REG_B32(GPIOD);
	REG_B32(GPIOE);
	REG_B32(GPIOF);
	REG_R(26);
REGDEF_BLOCK_END(rcc, ioport);

REGDEF_BLOCK_BEGIN()
	REG_B32(DMA1);
	REG_B32(DMA2);
	REG_R(6);
	REG_B32(FLASH);
	REG_R(3);
	REG_B32(CRC);
	REG_R(19);
REGDEF_BLOCK_END(rcc, ahb);

REGDEF_BLOCK_BEGIN()
	REG_RB();
	REG_B32(TIM3);
	REG_B32(TIM4);
	REG_RB();
	REG_B32(TIM6);
	REG_B32(TIM7);
	REG_R(2);
	REG_B32(UART5);
	REG_B32(UART6);
	REG_R(3);
	REG_B32(USB);
	REG_B32(SPI2);
	REG_B32(SPI3);
	REG_RB();
	REG_B32(USART2);
	REG_B32(USART3);
	REG_B32(USART4);
	REG_RB();
	REG_B32(I2C1);
	REG_B32(I2C2);
	REG_B32(I2C3);
	REG_R(3);
	REG_B32(DBG);
	REG_B32(PWREN);
	REG_R(3);
REGDEF_BLOCK_END(rcc, apb1);

REGDEF_BLOCK_BEGIN()
	REG_B32(SYSCFGEN);
	REG_R(10);
	REG_B32(TIM1);
	REG_B32(SPI1);
	REG_RB();
	REG_B32(USART1);
	REG_B32(TIM14);
	REG_B32(TIM15);
	REG_B32(TIM16);
	REG_B32(TIM17);
	REG_RB();
	REG_B32(ADC);
	REG_R(11);
REGDEF_BLOCK_END(rcc, apb2);

REGDEF_BLOCK_BEGIN()
	REG_K32(USART1SEL,2);
	REG_K32(USART2SEL,2);
	REG_K32(USART3SEL,2);
	REG_R(6);
	REG_K32(I2C1SEL,2);
	REG_K32(I2C2I2S1SEL,2);
	REG_R(6);
	REG_B32(TIM1SEL);
	REG_RB();
	REG_B32(TIM15SEL);
	REG_R(5);
	REG_K32(ADCSEL,2);
REGDEF_BLOCK_END(rcc, ccipr);

REGDEF_BLOCK_BEGIN()
	REG_K32(I2S1SEL,2);
	REG_K32(I2S2SEL,2);
	REG_R(8);
	REG_K32(USBSEL,2);
	REG_R(18);
REGDEF_BLOCK_END(rcc, ccipr2);

REGDEF_BLOCK_BEGIN()
	REG_B32(LSION);
	REG_B32(LSIRDY);
	REG_R(21);
	REG_B32(RMVF);
	REG_RB();
	REG_B32(OBLRSTF);
	REG_B32(PINRSTF);
	REG_B32(PORRSTF);
	REG_B32(SFTRSTF);
	REG_B32(IWDGRSTF);
	REG_B32(WWDGRSTF);
	REG_B32(LPWRSTF);
REGDEF_BLOCK_END(rcc, csr)

typedef struct STM32G070_STRUCT_NAME(Rcc) {
    /* Inherited -- MUST MATCH stm32_rcc.h. NO EXCEPTIONS. */
    STM32COMRccState parent;

    /* Additional clocks */
    Clk_t HSISYS,
	PLLCLK, PLLPCLK, PLLQCLK, PLLRCLK,
    SYSCLK,
    HCLK, /* Output from AHB Prescaler */
	HCLK8,
	PCLK,

    TIMPCLK; /* timer clock */

	union {
		struct {
			REGDEF_NAME(rcc,cr) 		CR; 		//0x00
			REGDEF_NAME(rcc,icscr) 		ICSCR;		//0x04
			REGDEF_NAME(rcc,cfgr) 		CFGR; 		//0x08
			REGDEF_NAME(rcc,pllcfgr) 	PLLCFGR; 	//0x0C
			REGDEF_R(0x10);
			REGDEF_R(0x14);
			REGDEF_NAME(rcc, inter) 	CIER; 		//0x18
			REGDEF_NAME(rcc, inter) 	CIFR; 		//0x1C
			REGDEF_NAME(rcc, inter) 	CICR; 		//0x20
			REGDEF_NAME(rcc, ioport) 	IOPRSTR;	//0x24
			REGDEF_NAME(rcc, ahb) 		AHBRSTR;	//0x28
			REGDEF_NAME(rcc, apb1) 		APBRSTR1;	//0x2C
			REGDEF_NAME(rcc, apb2) 		APBRSTR2;	//0x30
			REGDEF_NAME(rcc, ioport) 	IOPENR;		//0x34
			REGDEF_NAME(rcc, ahb) 		AHBENR;		//0x38
			REGDEF_NAME(rcc, apb1) 		APBENR1;	//0x3C
			REGDEF_NAME(rcc, apb2) 		APBENR2;	//0x40
			REGDEF_NAME(rcc, ioport) 	IOPSMENR;	//0x44
			REGDEF_NAME(rcc, ahb) 		AHBSMENR;	//0x48
			REGDEF_NAME(rcc, apb1) 		APBSMENR1;	//0x4C
			REGDEF_NAME(rcc, apb2) 		APBSMENR2;	//0x50
			REGDEF_NAME(rcc, ccipr)		CCIPR;		//0x54
			REGDEF_NAME(rcc, ccipr2)	CCIPR2;		//0x58
			REGDEF_NAME(rcc_com, bdcr)	BDCR; 		//0x5C
			REGDEF_NAME(rcc, csr) 		CSR;		//0x60

		} QEMU_PACKED;
		uint32_t raw[RI_END];
	} regs;

	qemu_irq* hclk_upd_irq;

} STM32G070_STRUCT_NAME(Rcc);


enum sw_src
{
	SW_HSISYS,
	SW_HSE,
	SW_PLLRCLK,
	SW_LSI,
	SW_LSE,
};

static const stm32_reginfo_t stm32g070_rcc_reginfo[RI_END] =
{
	[RI_CR] = {.mask = 0x30F3F00, .reset_val = 0x500},
	[RI_ICSCR] = {.mask = 0x7FFF, .reset_val = 0x4000, .unimp_mask = 0x7FFF},
	[RI_CFGR] = {.mask = 0xFFFF7F3F, .unimp_mask = 0xFFFF0000},
	[RI_PLLCFGR] = {.mask = 0xFF3F7F73, .reset_val =  0x1000},
	[RI_RES1 ... RI_RES2] = {.is_reserved = true},
	[RI_CIER] = {.mask = 0b111011},
	[RI_CIFR] = {.mask = 0b1100111011},
	[RI_CICR] = {.mask = 0b1100111011},
	[RI_IOPRSTR] = {.mask = 0x3F},
	[RI_IOPENR] = {.mask = 0x3F},
	[RI_IOPSMENR] = {.mask = 0x3F, .reset_val = 0x3F, .unimp_mask = 0x3F},
	[RI_AHBRSTR] = {.mask = 0x1103},
	[RI_AHBENR] = {.mask = 0x1103},
	[RI_AHBSMENR] = {.mask = 0x1103, .reset_val = 0x1103, .unimp_mask = 0x1103},
	[RI_APB1RSTR] = { .mask = 0x18EEE336},
	[RI_ABP1ENR] = { .mask = 0x18EEE336},
	[RI_ABP1SMENR] = { .mask = 0x18EEE336, .reset_val = 0x18EEE336, .unimp_mask = 0x18EEE336},
	[RI_APB2RSTR] = {.mask = 0x17D801},
	[RI_APB2ENR] = {.mask = 0x17D801},
	[RI_APB2SMENR] = {.mask = 0x17D801, .reset_val = 0x17D801, .unimp_mask = 0x17D801},
	[RI_CCIPR] = { .mask = 0xC140F03F},
	[RI_CCIPR2] = {.is_reserved = true, .mask = 0x300F},
	[RI_BDCR] = {.mask = 0x301837F, .unimp_mask = ~0x8303},
	[RI_CSR] = {.mask = 0xFE800003}
};

/* REGISTER IMPLEMENTATION */

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc,cr) new = { .raw = new_value };

    if((clktree_is_enabled(&s->PLLCLK) && !new.PLLON && !init) &&
       s->regs.CFGR.SW == SW_PLLRCLK) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(&s->PLLCLK, new.PLLON);
	s->regs.CR.PLLRDY = s->regs.CR.PLLON = new.PLLON;

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.HSEON && !init) &&
       (s->regs.CFGR.SW == SW_HSE || s->regs.CFGR.SW == SW_PLLRCLK)
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSECLK, new.HSEON);
	s->regs.CR.HSERDY = s->regs.CR.HSEON = new.HSEON;

	clktree_set_scale(&s->HSISYS, 1, 1U<<new.HSIDIV);

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.HSEON && !init) &&
       (s->regs.CFGR.SW == SW_HSISYS || s->regs.CFGR.SW == SW_PLLRCLK)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSICLK, new.HSION);
	s->regs.CR.HSIRDY = s->regs.CR.HSION = new.HSION;
}

static void stm32_rcc_RCC_CFGR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc,cfgr) new = { .raw = new_value };
	// SW
	if (new.SW > 4)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid clock source for SYSCLK selected!");
	}
	else
	{
		clktree_set_selected_input(&s->SYSCLK, new.SW);
		s->regs.CFGR.SWS = s->regs.CFGR.SW = new.SW;
	}

	// HPRE:
	clktree_set_scale(&s->HCLK, 1, AHBPRE_OPTS[new.HPRE]);
	s->regs.CFGR.HPRE = new.HPRE;

    /* PPRE */
    s->regs.CFGR.PPRE = new.PPRE;
	clktree_set_scale(&s->PCLK, 1, APBPRE_OPTS[new.PPRE]);

	clktree_set_scale(&s->TIMPCLK, (APBPRE_OPTS[new.PPRE] == 1U ? 1U : 2U), 1);
}

static void stm32_rcc_RCC_PLLCFGR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc,pllcfgr) new = { .raw = new_value };
	const REGDEF_NAME(rcc,pllcfgr) changed = { .raw = new_value^s->regs.PLLCFGR.raw };
	bool pll_on = s->regs.CR.PLLON;
	clktree_set_selected_input(&s->PLLCLK, (new.PLLSRC - 2));
	if (new.PLLN < 0b1000 || new.PLLN > 0b1010110 || pll_on )
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid PLL multiplier (N) selected or tried to change it while PLL enabled!");
	}
	else
	{
		clktree_set_scale(&s->PLLCLK, new.PLLN, new.PLLM + 1U);
	}

	bool pll_changed = pll_on && (
		changed.PLLSRC || changed.PLLQ || changed.PLLP || changed.PLLR
	);

	if (pll_changed)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Attempted to change PLL configuration while it is in use!");
	}

	clktree_set_enabled(&s->PLLPCLK, new.PLLPEN);
	if (new.PLLQ == 0 && changed.PLLQ)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid PLLQ divisor selected!");
	}
	else
	{
		clktree_set_enabled(&s->PLLQCLK, new.PLLQEN);
		clktree_set_scale(&s->PLLQCLK, 1, new.PLLQ + 1U);
	}
	if (new.PLLR == 0 && changed.PLLR)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid PLLR divisor selected!");
	}
	else
	{
		clktree_set_enabled(&s->PLLRCLK, new.PLLREN);
		clktree_set_scale(&s->PLLRCLK, 1, new.PLLR + 1U);
	}
	if (new.PLLP == 0 && changed.PLLP)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Invalid PLLP divisor selected!");
	}
	else
	{
		clktree_set_enabled(&s->PLLPCLK, new.PLLPEN);
		clktree_set_scale(&s->PLLPCLK, 1, new.PLLP + 1U);
	}
	s->regs.PLLCFGR.raw = new.raw;
}

static void stm32_rcc_RCC_CCIPR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(rcc,ccipr) new = { .raw = new_value };

	clktree_set_selected_input(&s->parent.pclocks[STM32_P_UART1], new.USART1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_UART2], new.USART2SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_UART3], new.USART3SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2C1], new.I2C1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2C2], new.I2C2I2S1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_TIM1], new.TIM1SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_TIM15], new.TIM15SEL);
	clktree_set_selected_input(&s->parent.pclocks[STM32_P_ADC1], new.ADCSEL);
	s->regs.CCIPR.raw = new.raw;
}

static void stm32_rcc_RCC_BDCR_writeb0(STM32G070_STRUCT_NAME(Rcc) *s, uint8_t new_value, bool init)
{
	REGDEF_NAME(rcc_com,bdcr) val = {.raw = new_value};
    clktree_set_enabled(&s->parent.LSECLK, val.LSEON);
	s->regs.BDCR.LSCOEN = s->regs.BDCR.LSERDY = val.LSEON;


	if (s->regs.BDCR.RTC_SEL)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Cannot change RTC clock source after it has been set!");
	}
	else if (val.RTC_SEL == 0)
	{
		clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], 0);
	}
	else
	{
		clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], val.RTCEN);
		clktree_set_selected_input(&s->parent.pclocks[STM32_P_RTC], val.RTC_SEL - 1U );
	}

}

static void stm32_rcc_RCC_BDCR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
    stm32_rcc_RCC_BDCR_writeb0(s, new_value & 0xff, init);
}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(STM32G070_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	REGDEF_NAME(rcc,csr) val = { .raw = new_value };
    clktree_set_enabled(&s->parent.LSICLK, val.LSION);
	val.LSIRDY = val.LSION;
	s->regs.raw[RI_CSR] = val.raw;
}



static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(opaque);
	uint32_t index = offset >> 2U;
	CHECK_BOUNDS_R(index, RI_END, stm32g070_rcc_reginfo, "RCC");
	//rather than reconstruct the register each read, I ensure the stored value is current when changes happen.
	return s->regs.raw[index];
}

static void stm32_rcc_writeb(void *opaque, hwaddr offset, uint64_t value)
{
    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(opaque);

    switch (offset>>2U) {
    case RI_BDCR:
        stm32_rcc_RCC_BDCR_writeb0(s, value, false);
        break;
    default:
        STM32_BAD_REG(offset, 1U);
        break;
    }
}

static void stm32_rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value, bool is_reset)
{
    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(opaque);

	uint8_t index = offset>>2U;

    switch(index) {
        case RI_CR:
            stm32_rcc_RCC_CR_write(s, value, is_reset);
            break;
        case RI_CFGR:
            stm32_rcc_RCC_CFGR_write(s, value, is_reset);
            break;
        case RI_PLLCFGR:
            stm32_rcc_RCC_PLLCFGR_write(s, value, is_reset);
            break;
        case RI_CCIPR:
            stm32_rcc_RCC_CCIPR_write(s, value, is_reset);
            break;
		case RI_IOPRSTR ... RI_APB2RSTR:
			stm32_common_rcc_reset_write(&s->parent,value, PERIPH_BLOCKS[index - RI_IOPRSTR]);
			s->regs.raw[index] = value;
			break;
		case RI_IOPENR ... RI_APB2ENR:
            stm32_common_rcc_enable_write(&s->parent, value, PERIPH_BLOCKS[index - RI_IOPENR]);
			s->regs.raw[index] = value;
            break;
		case RI_IOPSMENR ... RI_APB2SMENR:
			//printf("STOP MODE rcc write not implemetned!\n");
			s->regs.raw[index] = value;
			break;
        case RI_BDCR:
            stm32_rcc_RCC_BDCR_write(s, value, is_reset);
            break;
        case RI_CSR:
            stm32_rcc_RCC_CSR_write(s, value, is_reset);
            break;
			break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            stm32_unimp("Unimplemented: RCC read from register at offset %08" HWADDR_PRIx, offset);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value, false);
            break;
        case 1:
            stm32_rcc_writeb(opaque, offset, value);
            break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
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
    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		stm32_rcc_writew(dev, i<<2U, stm32g070_rcc_reginfo[i].reset_val, true);
	}
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    STM32G070_STRUCT_NAME(Rcc) *s = (STM32G070_STRUCT_NAME(Rcc) *)opaque;

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
    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(dev);

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

	clktree_create_clk(&s->HSISYS, "HSISYS", 1, 1, true, s->parent.hsi_freq, 0, &s->parent.HSICLK, NULL);

	// TODO - determine max sysclk freq.
    clktree_create_clk(&s->SYSCLK, "SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   &s->HSISYS, &s->parent.HSECLK, &s->PLLRCLK, &s->parent.LSICLK, &s->parent.LSECLK, NULL);

    clktree_create_clk(&s->HCLK, "HCLK", 1, 1, true, 64000000, 0, &s->SYSCLK, NULL);
    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);
    clktree_create_clk(&s->HCLK8, "HCLK8", 1, 8, true, 8000000, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PCLK, "PCLK", 1, 1, true, 64000000, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PLLCLK, "PLLCLK", 1, 2, true, 48000000, 0, &s->parent.HSICLK, &s->parent.HSECLK,  NULL);
	clktree_create_clk(&s->PLLPCLK, "PLLP", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLLCLK, NULL);
	clktree_create_clk(&s->PLLQCLK, "PLLQ", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLLCLK, NULL);
	clktree_create_clk(&s->PLLRCLK, "PLLR", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLLCLK, NULL);

	clktree_create_clk(&s->TIMPCLK, "TIMPCLK", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);

    /* AHB Peripheral clocks */
    INIT_PCLK(GPIOA, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOB, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOD, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOE, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(GPIOF, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);


// verified
    INIT_PCLK(CRC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(FSMC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(EXTI, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(RCC, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMAMUX, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMA1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    INIT_PCLK(DMA2, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    INIT_PCLK(UART1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);
    INIT_PCLK(UART2, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);
    INIT_PCLK(UART3, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->SYSCLK, &s->parent.HSICLK, &s->parent.LSECLK, NULL);

    INIT_PCLK(I2C1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->parent.LSECLK, &s->parent.HSICLK, &s->SYSCLK, NULL);
    INIT_PCLK(I2C2, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, &s->parent.LSECLK, &s->parent.HSICLK, &s->SYSCLK, NULL);

    INIT_PCLK(TIM1,  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, &s->PLLQCLK, NULL);
    INIT_PCLK(TIM15,1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, &s->PLLQCLK, NULL);

    INIT_PCLK(ADC1, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->SYSCLK, &s->PLLPCLK, &s->parent.HSICLK, NULL);

	// Everything else is APB

    INIT_PCLK(SYSCFG, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(UART4, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(UART5, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(UART6, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK, NULL);
    INIT_PCLK(TIM3,   1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM6,   1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM7,   1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);;
    INIT_PCLK(TIM14, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM16, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
    INIT_PCLK(TIM17, 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->TIMPCLK, NULL);
// vf'd end


    INIT_PCLK(IWDG, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.LSICLK, NULL);
    INIT_PCLK(PWR, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->SYSCLK, NULL);
    INIT_PCLK(RTC, 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.LSICLK, &s->parent.LSECLK, &s->parent.HSECLK, NULL);

}



static void stm32_rcc_init(Object *obj)
{

    STM32G070_STRUCT_NAME(Rcc) *s = STM32G070_RCC(obj);

    STM32_MR_IO_INIT(&s->parent.iomem, obj, &stm32_rcc_ops, s, 1U * KiB);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->parent.iomem);
}

static const VMStateDescription vmstate_stm32f2xx_rcc = {
    .name = TYPE_STM32G070_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(parent.pclocks, STM32G070_STRUCT_NAME(Rcc), STM32_P_COUNT, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(SYSCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLPCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLQCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLRCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK,STM32G070_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
		VMSTATE_UINT32_ARRAY(regs.raw, STM32G070_STRUCT_NAME(Rcc), RI_END),
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
    .name  = TYPE_STM32G070_RCC,
    .parent = TYPE_STM32COM_RCC,
    .instance_size  = sizeof(STM32G070_STRUCT_NAME(Rcc)),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init,
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
