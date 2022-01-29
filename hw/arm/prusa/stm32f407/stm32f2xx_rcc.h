/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Portions Copyright (C) 2010 Andre Beckus
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>

 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
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
#include "qemu-common.h"
#include "stm32_clktree.h"
#include "../stm32_common/stm32_clk.h"
#include "../stm32_common/stm32_rcc.h"
#include "../stm32_common/stm32_common.h"
#include "stm32.h"

REGDEF_BLOCK_BEGIN()
		REG_B32(HSION);
		REG_B32(HSIRDY);
		REG_RB();
		REG_K32(HSITRIM ,5);
		REG_K32(HSICAL  ,8);
		REG_B32(HSEON);
		REG_B32(HSERDY);
		REG_B32(HSEBYP);
		REG_B32(CSSON);
		REG_R(4);
		REG_B32(PLLON);
		REG_B32(PLLRDY);
		REG_B32(PLLI2SON);
		REG_B32(PLLI2SRDY);
		REG_R(4);
REGDEF_BLOCK_END(rcc,cr);

REGDEF_BLOCK_BEGIN()
	REG_B32(LSEON);
	REG_B32(LSERDY);
	REG_B32(LSEBYP);
	REG_K32(LSEDRV,2);
	REG_B32(LSECSSON); 
	REG_B32(LSECCSD);  
	REG_RB();
	REG_K32(RTC_SEL,2);
	REG_R(5);
	REG_B32(RTCEN);
	REG_B32(BDRST);
	REG_R(7);
	REG_B32(LSCOEN);
	REG_B32(LSCOSEL);
	REG_R(6);
REGDEF_BLOCK_END(rcc, bdcr)

REGDEF_BLOCK_BEGIN()
	REG_B32(LSION);
	REG_B32(LSIRDY);
	REG_R(22);
	REG_B32(RMVF);
	REG_B32(BORRSTF);
	REG_B32(PADRSTF);
	REG_B32(PORRSTF);
	REG_B32(SFTRSTF);
	REG_B32(IWDGRSTF);
	REG_B32(WWDGRSTF);
	REG_B32(LPWRSTF);
REGDEF_BLOCK_END(rcc, csr)

REGDEF_BLOCK_BEGIN()
		REG_K32(SW,2);
		REG_K32(SWS,2);
		REG_K32(HPRE,4);
		REG_R(2);
		REG_K32(PPRE1,3);
		REG_K32(PPRE2,3);
		REG_K32(RTCPRE,5);
		REG_K32(MCO1,2);
		REG_B32(I2SSRC);
		REG_K32(MCO1PRE,3);
		REG_K32(MCO2PRE,3);
		REG_K32(MCO2,2);
REGDEF_BLOCK_END(rcc, cfgr);

REGDEF_BLOCK_BEGIN()
		REG_K32(PLLM,6);
		REG_K32(PLLN,9);
		REG_RB();
		REG_K32(PLLP,2);
		REG_R(4);
		REG_B32(PLLSRC);
		REG_K32(PLLQ,4);
		REG_R(4);
REGDEF_BLOCK_END(rcc, pllcfgr);

REGDEF_BLOCK_BEGIN()
		REG_R(6);
		REG_K32(PLLN,9);
		REG_R(12);
		REG_K32(PLLI2SR,3);
		REG_RB();
REGDEF_BLOCK_END(rcc, plli2scfgr);

OBJECT_DECLARE_SIMPLE_TYPE(Stm32f2xxRcc, STM32F2xx_RCC)

#define R_RCC_MAX (0x88/4)

typedef struct Stm32f2xxRcc {
    /* Inherited -- MUST MATCH stm32_rcc.h. NO EXCEPTIONS. */
    STM32COMRccState parent;

    /* Additional clocks */
    Clk_t
    SYSCLK,
    RTCCLK, // special one that applies the CFGR divisor
	TIMCLK1, // handles the timer prescale based on APB1 pre
	TIMCLK2, // handles the timer prescale based on APB2 pre

    PLLM, /* Applies "M" division and "N" multiplication factors for PLL */
    PLLCLK,
    PLL48CLK,

    PLLI2SM, /* Applies "M" division and "N" multiplication factors for PLLI2S */
    PLLI2SCLK,

    HCLK, /* Output from AHB Prescaler */
    PCLK1, /* Output from APB1 Prescaler */
    PCLK2; /* Output from APB2 Prescaler */

    /* Register Values */

	union {
		struct {
			REGDEF_NAME(rcc,cr) CR;
			REGDEF_NAME(rcc,pllcfgr) PLLCFGR;
			REGDEF_NAME(rcc,cfgr) CFGR;
			uint32_t CIR;
			uint32_t AHB1RSTR;
			uint32_t AHB2RSTR;
			uint32_t AHB3RSTR;
			REGDEF_R(0x1C);
			uint32_t APB1RSTR;
			uint32_t APB2RSTR;
			REGDEF_R(0x28);
			REGDEF_R(0x2C);
			uint32_t AHB1ENR;
			uint32_t AHB2ENR;
			uint32_t AHB3ENR;
			REGDEF_R(0x3C);
			uint32_t APB1ENR;
			uint32_t APB2ENR;
			REGDEF_R(0x48);
			REGDEF_R(0x4C);
			uint32_t AHB1LPENR;
			uint32_t AHB2LPENR;
			uint32_t AHB3LPENR;
			REGDEF_R(0x5C);
			uint32_t APB1LPENR;
			uint32_t APB2LPENR;
			REGDEF_R(0x68);
			REGDEF_R(0x6C);
			REGDEF_NAME(rcc,bdcr) BDCR;
			REGDEF_NAME(rcc, csr) CSR;
			REGDEF_R(0x78);
			REGDEF_R(0x7C);
			uint32_t SSCGR;
			REGDEF_NAME(rcc, plli2scfgr) PLLI2SCFGR;
		} QEMU_PACKED;
		uint32_t raw[R_RCC_MAX];
	} regs;

    qemu_irq *hclk_upd_irq;
} Stm32f2xxRcc;
