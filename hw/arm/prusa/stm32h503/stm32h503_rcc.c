/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 * (STM32H503x variants)
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
#include "../stm32_registers/generated/stm32h503/RCC_index.h"
#include "../stm32_registers/generated/stm32h503/RCC_registers.h"

/* DEFINITIONS*/

/* See README for DEBUG details. */
//#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32F2XX_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


QEMU_BUILD_BUG_MSG(STM32_P_COUNT>255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

static const uint8_t AHB1_PERIPHS[32] = {
    STM32_P_DMA1, STM32_P_DMA2, 0, 0, 0, 0, 0, 0,
    STM32_P_FSMC, 0, 0, 0, STM32_P_CRC, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t AHB2_PERIPHS[32] = {
    STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, 0, 0, 0, STM32_P_GPIOH,
    0, 0, STM32_P_ADC1, STM32_P_DAC, 0, 0, 0, 0,
    0, STM32_P_HASH, STM32_P_RNG, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0 /*STM32_P_SRAM2*/, 0
};


static const uint8_t APB1L_PERIPHS[32] = {
    STM32_P_TIM2, STM32_P_TIM3, 0, 0, STM32_P_TIM6, STM32_P_TIM7, 0, 0,
    0, 0, 0, STM32_P_WWDG, 0, 0/*opamp*/, STM32_P_SPI2, STM32_P_SPI3,
    0 /*COMP*/, STM32_P_USART2, STM32_P_USART3, 0, 0, STM32_P_I2C1, STM32_P_I2C2, STM32_P_I3C1,
    0 /*CRS*/, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB1H_PERIPHS[32] = {
    0, 0, 0, 0/*DTSEN*/, 0, 0/*LP2TIM*/, 0, 0,
    0, 0/*FDCAN*/, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB2_PERIPHS[32] = {
    STM32_P_SYSCFG, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_USART1, STM32_P_TIM14,
    STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, STM32_P_ADC1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB3_PERIPHS[32] = {
    0, 0/*SBS*/, 0, 0, 0, 0, 0/*LPUART1*/, 0,
    0, STM32_P_I3C2, 0, STM32_P_LPTIM1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t (*PERIPH_BLOCKS[])[32] = {
	[RI_AHB1ENR - RI_AHB1ENR] = &AHB1_PERIPHS,
	[RI_AHB2ENR - RI_AHB1ENR] = &AHB2_PERIPHS,
	[RI_APB1LENR - RI_AHB1ENR] = &APB1L_PERIPHS,
	[RI_APB1HENR - RI_AHB1ENR] = &APB1H_PERIPHS,
	[RI_APB2ENR - RI_AHB1ENR] = &APB2_PERIPHS,
	[RI_APB3ENR - RI_AHB1ENR] = &APB3_PERIPHS,
};

// static const uint16_t AHBPRE_OPTS[16] = {
// 	[0b0000 ... 0b0111] = 1,
// 	[0b1000] = 2,
// 	[0b1001] = 4,
// 	[0b1010] = 8,
// 	[0b1011] = 16,
// 	[0b1100] = 64,
// 	[0b1101] = 128,
// 	[0b1110] = 256,
// 	[0b1111] = 512
// };

// static const uint8_t APBPRE_OPTS[8] = {
// 	[0b000 ... 0b011] = 1,
// 	[0b100] = 2,
// 	[0b101] = 4,
// 	[0b110] = 8,
// 	[0b111] = 16
// };

OBJECT_DECLARE_SIMPLE_TYPE(STM32H503_STRUCT_NAME(Rcc), STM32H503_RCC);

  	// RI_CFGR1         = 0x1C/4, /*!< RCC clock configuration register 1                   */
  	// RI_CFGR2         = 0x20/4, /*!< RCC clock configuration register 2                   */
  	// RI_PLL1CFGR      = 0x28/4, /*!< RCC PLL1 Configuration Register                      */
  	// RI_PLL2CFGR      = 0x2C/4, /*!< RCC PLL2 Configuration Register                      */
  	// RI_PLL1DIVR      = 0x34/4, /*!< RCC PLL1 Dividers Configuration Register             */
  	// RI_PLL1FRACR     = 0x38/4, /*!< RCC PLL1 Fractional Divider Configuration Register   */
  	// RI_PLL2DIVR      = 0x3C/4, /*!< RCC PLL2 Dividers Configuration Register             */
  	// RI_PLL2FRACR     = 0x40/4, /*!< RCC PLL2 Fractional Divider Configuration Register   */
  	// RI_CIER          = 0x50/4, /*!< RCC Clock Interrupt Enable Register                  */
  	// RI_CIFR          = 0x54/4, /*!< RCC Clock Interrupt Flag Register                    */
  	// RI_CICR          = 0x58/4, /*!< RCC Clock Interrupt Clear Register                   */
  	// RI_AHB1RSTR      = 0x60/4, /*!< RCC AHB1 Peripherals Reset Register                  */
  	// RI_AHB2RSTR      = 0x64/4, /*!< RCC AHB2 Peripherals Reset Register                  */
  	// RI_APB1LRSTR     = 0x74/4, /*!< RCC APB1 Peripherals reset Low Word register         */
  	// RI_APB1HRSTR     = 0x78/4, /*!< RCC APB1 Peripherals reset High Word register        */
  	// RI_APB2RSTR      = 0x7C/4, /*!< RCC APB2 Peripherals Reset Register                  */
  	// RI_APB3RSTR      = 0x80/4, /*!< RCC APB3 Peripherals Reset Register                  */
  	// RI_AHB1ENR       = 0x88/4, /*!< RCC AHB1 Peripherals Clock Enable Register           */
  	// RI_AHB2ENR       = 0x8C/4, /*!< RCC AHB2 Peripherals Clock Enable Register           */
  	// RI_APB1LENR      = 0x9C/4, /*!< RCC APB1 Peripherals clock Enable Low Word register  */
  	// RI_APB1HENR      = 0xA0/4, /*!< RCC APB1 Peripherals clock Enable High Word register */
  	// RI_APB2ENR       = 0xA4/4, /*!< RCC APB2 Peripherals Clock Enable Register           */
  	// RI_APB3ENR       = 0xA8/4, /*!< RCC APB3 Peripherals Clock Enable Register           */
  	// RI_AHB1LPENR     = 0xB0/4, /*!< RCC AHB1 Peripheral sleep clock Register             */
  	// RI_AHB2LPENR     = 0xB4/4, /*!< RCC AHB2 Peripheral sleep clock Register             */
  	// RI_APB1LLPENR    = 0xC4/4, /*!< RCC APB1 Peripherals sleep clock Low Word Register   */
  	// RI_APB1HLPENR    = 0xC8/4, /*!< RCC APB1 Peripherals sleep clock High Word Register  */
  	// RI_APB2LPENR     = 0xCC/4, /*!< RCC APB2 Peripherals sleep clock Register            */
  	// RI_APB3LPENR     = 0xD0/4, /*!< RCC APB3 Peripherals Clock Low Power Enable Register */
  	// RI_CCIPR1        = 0xD8/4, /*!< RCC IPs Clocks Configuration Register 1              */
  	// RI_CCIPR2        = 0xDC/4, /*!< RCC IPs Clocks Configuration Register 2              */
  	// RI_CCIPR3        = 0xE0/4, /*!< RCC IPs Clocks Configuration Register 3              */
  	// RI_CCIPR4        = 0xE4/4, /*!< RCC IPs Clocks Configuration Register 4              */
  	// RI_CCIPR5        = 0xE8/4, /*!< RCC IPs Clocks Configuration Register 5              */
  	// RI_BDCR          = 0xF0/4, /*!< RCC VSW Backup Domain & V33 Domain Control Register  */
  	// RI_RSR           = 0xF4/4, /*!< RCC Reset status Register                            */
  	// RI_PRIVCFGR      = 0x114/4,/*!< RCC Privilege configuration register                 */

typedef struct STM32H503_STRUCT_NAME(Rcc) {
    /* Inherited -- MUST MATCH stm32_rcc.h. NO EXCEPTIONS. */
    STM32COMRccState parent;

    /* Additional clocks */
	Clk_t
	DUMMY, /* Dummy clock that's off as a placeholder for "no clock"*/
    SYS_CK,
	SYSTICK, /* SysTick clock */
	HSI_KER_CK, /* Output from HSI DIV */
    HCLK, /* Output from AHB Prescaler */
	HCLK8, /* Output from AHB Prescaler/8 (for systick) */
	HCLK1, // AHB1 and 2 clock en/dis
	HCLK2,
	PCLK1, /* Output from APB1 Prescaler */
	PCLK2, /* Output from APB2 Prescaler */
	PCLK3, /* Output from APB3 Prescaler */
	PLL1CLK, /* Output from PLL1 */
	PLL1PCLK, /* Output from PLL1 PCLK */
	PLL1QCLK, /* Output from PLL1 QCLK */
	PLL1RCLK, /* Output from PLL1 RCLK */
	PLL2CLK, /* Output from PLL2 */
	PLL2PCLK, /* Output from PLL2 PCLK */
	PLL2QCLK, /* Output from PLL2 QCLK */
	PLL2RCLK, /* Output from PLL2 RCLK */
	RTCHSEPRE, /* Output from RTC HSE prescaler */
	TIM1PRE, /* Output from TIM1 doubler */
	TIM23PRE, /* Output from TIM23 doubler */
	AUDIOCLK,
	PER_CK,
	CSI_CK,
	HSI48_KER_CK; // For USB

// aliases for names per the clktree
#define LSE_CK parent.LSECLK
#define LSI_KER_CK parent.LSICLK
#define HSE_CK parent.HSECLK
#define HSI_CK parent.HSICLK

	/* Additional registers */

	stm32reg_h503_rcc_t regs;

	qemu_irq* hclk_upd_irq;

} STM32H503_STRUCT_NAME(Rcc);

static uint16_t h503_hclk_div_lut[0x10] = {
	[0b0000 ... 0b0111] = 1,
	[0b1000] = 2,
	[0b1001] = 4,
	[0b1010] = 8,
	[0b1011] = 16,
	[0b1100] = 64,
	[0b1101] = 128,
	[0b1110] = 256,
	[0b1111] = 512,
};

static uint8_t h503_pclk_div_lut[0x8] = {
	[0b000 ... 0b011] = 1,
	[0b100] = 2,
	[0b101] = 4,
	[0b110] = 8,
	[0b111] = 16,
};

enum sw_src
{
	SW_HSISYS,
	SW_CSI,
	SW_HSE,
	SW_PLL1,
};

enum rtc_src
{
	RTC_NONE,
	RTC_LSE,
	RTC_LSI,
	RTC_HSE,
};

enum pll_src
{
	PLL_DISABLED,
	PLL_HSI,
	PLL_CSI,
	PLL_HSE,
};

enum uart_src
{
	UART_PCLK,
	UART_PLL2Q,
	UART_HSI_KER,
	UART_CSI_KER,
	UART_LSE,
};

enum spi_src
{
	SPI_PLL1Q,
	SPI_PLL2P,
	SPI_AUDIO,
	SPI_PCLK,
};

enum i3c_sel
{
	I3C_PCLK3,
	I3C_PLL2R,
	I3C_HSI_KER,
	I3C_NONE,
};

enum i2s_sel
{
	I2S_PCLK1,
	I2S_PLL2R,
	I2S_HSI_KER,
	I2S_CSI_KER,
};

enum usb_src
{
	USB_NONE,
	USB_PLL1Q,
	USB_PLL2Q,
	USB_HSI48_KER,
};

enum systick_src
{
	SYSTICK_HCLK8,
	SYSTICK_LSI_KER_CK,
	SYSTICK_LSE_CK,
	SYSTICK_END,
};

enum analog_src
{
	ANALOG_HCLK,
	ANALOG_SYSCLK,
	ANALOG_PLL2R,
	ANALOG_HSE,
	ANALOG_KSI_KER,
	ANALOG_CSI_KER,
	ANALOG_END
};

/* REGISTER IMPLEMENTATION */
static void stm32h503_RCC_setup_pll(STM32H503_STRUCT_NAME(Rcc) *s, uint8_t pll_index)
{
	// Layout of the PLL registers is identical for both PLL1 and PLL2.
	const REGDEF_NAME(h503_rcc,pll1cfgr) *pll = (pll_index == 1 ? &s->regs.PLL1CFGR : (void*)&s->regs.PLL2CFGR);
	// const REGDEF_NAME(h503_rcc,pll1fracr) *pllfrac = (pll_index == 1 ? &s->regs.PLL1FRACR : &s->regs.PLL2FRACR);
	const REGDEF_NAME(h503_rcc,pll1divr) *plldiv = (pll_index == 1 ? &s->regs.PLL1DIVR : (void*)&s->regs.PLL2DIVR);
	Clk_t *pllclk = (pll_index == 1 ? &s->PLL1CLK : &s->PLL2CLK);
	Clk_t *pllpclk = (pll_index == 1 ? &s->PLL1PCLK : &s->PLL2PCLK);
	Clk_t *pllqclk = (pll_index == 1 ? &s->PLL1QCLK : &s->PLL2QCLK);
	Clk_t *pllrclk = (pll_index == 1 ? &s->PLL1RCLK : &s->PLL2RCLK);

	clktree_set_selected_input(pllclk, pll->bits.PLL1SRC);
	clktree_set_scale(pllclk, plldiv->bits.PLL1N + 1U, pll->bits.PLL1M);
	clktree_set_scale(pllpclk, 1, plldiv->bits.PLL1P + 1U);
	clktree_set_scale(pllqclk, 1, plldiv->bits.PLL1Q + 1U);
	clktree_set_scale(pllrclk, 1, plldiv->bits.PLL1R + 1U);
	clktree_set_enabled(pllpclk, pll->bits.PLL1PEN);
	clktree_set_enabled(pllqclk, pll->bits.PLL1QEN);
	clktree_set_enabled(pllrclk, pll->bits.PLL1REN);


}

static void stm32_rcc_RCC_CR_write(STM32H503_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(h503_rcc,cr) new = { .raw = new_value };

    if((clktree_is_enabled(&s->PLL1CLK) && !new.bits.PLL1ON && !init) &&
       s->regs.CFGR1.bits.SW == SW_PLL1) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }

	if (new.bits.PLL1ON & !s->regs.CR.bits.PLL1ON)
	{
		stm32h503_RCC_setup_pll(s, 1);
	}
    clktree_set_enabled(&s->PLL1CLK, new.bits.PLL1ON);
	s->regs.CR.bits.PLL1RDY = s->regs.CR.bits.PLL1ON = new.bits.PLL1ON;

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.bits.HSEON && !init) &&
       (s->regs.CFGR1.bits.SW == SW_HSE || s->regs.CFGR1.bits.SW == SW_PLL1)
	   // TODO - check if PLLSRC is HSE...
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSECLK, new.bits.HSEON);
	s->regs.CR.bits.HSERDY = s->regs.CR.bits.HSEON = new.bits.HSEON;

	clktree_set_scale(&s->HSI_KER_CK, 1, 1U<<new.bits.HSIDIV);

    if((clktree_is_enabled(&s->parent.HSECLK) && !new.bits.HSEON && !init) &&
       (s->regs.CFGR1.bits.SW == SW_HSISYS || s->regs.CFGR1.bits.SW == SW_PLL1)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSICLK, new.bits.HSION);
	s->regs.CR.bits.HSIRDY = s->regs.CR.bits.HSION = new.bits.HSION;

	if (new.bits.PLL2ON & !s->regs.CR.bits.PLL2ON)
	{
		stm32h503_RCC_setup_pll(s, 2);
	}
	clktree_set_enabled(&s->PLL2CLK, new.bits.PLL2ON);
	s->regs.CR.bits.PLL2RDY = s->regs.CR.bits.PLL2ON = new.bits.PLL2ON;

	clktree_set_enabled(&s->CSI_CK, new.bits.CSION);
	s->regs.CR.bits.CSIRDY = s->regs.CR.bits.CSION = new.bits.CSION;

	clktree_set_enabled(&s->HSI48_KER_CK, new.bits.HSI48ON);
	s->regs.CR.bits.HSI48RDY = s->regs.CR.bits.HSI48ON = new.bits.HSI48ON;
}

static void stm32_rcc_RCC_CFGR1_write(STM32H503_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
	const REGDEF_NAME(h503_rcc,cfgr1) new = { .raw = new_value };

	clktree_set_selected_input(&s->SYS_CK, new.bits.SW);
	s->regs.CFGR1.bits.SWS = s->regs.CFGR1.bits.SW = new.bits.SW;

	// HPRE:
	clktree_set_enabled(&s->RTCHSEPRE, new.bits.RTCPRE < 2);
	clktree_set_scale(&s->RTCHSEPRE, 1, MAX(1,new.bits.RTCPRE));
	s->regs.CFGR1.bits.RTCPRE = new.bits.RTCPRE;

	if (new.bits.TIMPRE)
	{
		clktree_set_scale(&s->TIM23PRE, (s->regs.CFGR2.bits.PPRE1 > 0b101 ? 4 : 2), 1);
		clktree_set_scale(&s->TIM1PRE, (s->regs.CFGR2.bits.PPRE2 > 0b101 ? 4 : 2), 1);
	}
	else
	{
		clktree_set_scale(&s->TIM23PRE, (s->regs.CFGR2.bits.PPRE1 > 0b100 ? 2 : 1), 1);
		clktree_set_scale(&s->TIM1PRE, (s->regs.CFGR2.bits.PPRE2 > 0b100 ? 2 : 1), 1);
	}
}

static void stm32_rcc_RCC_BDCR_writeb0(STM32H503_STRUCT_NAME(Rcc) *s, uint8_t new_value, bool init)
{
	REGDEF_NAME(rcc_com,bdcr) val = {.raw = new_value};
    clktree_set_enabled(&s->parent.LSECLK, val.LSEON);
	s->regs.BDCR.bits.LSCOEN = s->regs.BDCR.bits.LSERDY = val.LSEON;


	if (s->regs.BDCR.bits.RTCSEL)
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

static void stm32_rcc_RCC_BDCR_write(STM32H503_STRUCT_NAME(Rcc) *s, uint32_t new_value, bool init)
{
    stm32_rcc_RCC_BDCR_writeb0(s, new_value & 0xff, init);
}

static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(opaque);
	uint32_t index = offset >> 2U;
	CHECK_BOUNDS_R(index, RI_END, stm32_h503_rcc_reginfo, "RCC");
	//rather than reconstruct the register each read, I ensure the stored value is current when changes happen.
	return s->regs.raw[index];
}

static void stm32_rcc_writeb(void *opaque, hwaddr offset, uint64_t value)
{
    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(opaque);

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
    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(opaque);

	uint8_t index = offset>>2U;

	CHECK_UNIMP_RESVD_V2(value, s->regs.raw[index], stm32_h503_rcc_reginfo, index);

    switch(index) {
        case RI_CR:
            stm32_rcc_RCC_CR_write(s, value, is_reset);
            break;
        case RI_CFGR1:
            stm32_rcc_RCC_CFGR1_write(s, value, is_reset);
            break;
		case RI_CFGR2:
			{
				const REGDEF_NAME(h503_rcc,cfgr2) new = { .raw = value };
				clktree_set_scale(&s->HCLK, 1, h503_hclk_div_lut[new.bits.HPRE]);
				clktree_set_scale(&s->PCLK1, 1, h503_pclk_div_lut[new.bits.PPRE1]);
				clktree_set_scale(&s->PCLK2, 1, h503_pclk_div_lut[new.bits.PPRE2]);
				clktree_set_scale(&s->PCLK3, 1, h503_pclk_div_lut[new.bits.PPRE3]);
				clktree_set_enabled(&s->PCLK1, !new.bits.APB1DIS);
				clktree_set_enabled(&s->PCLK2, !new.bits.APB2DIS);
				clktree_set_enabled(&s->PCLK3, !new.bits.APB3DIS);
				clktree_set_enabled(&s->HCLK1, !new.bits.AHB1DIS);
				clktree_set_enabled(&s->HCLK2, !new.bits.AHB2DIS);
				s->regs.CFGR2.raw = new.raw;
			}
			break;
        case RI_PLL1CFGR:
		case RI_PLL1DIVR:
		case RI_PLL1FRACR:
			if (s->regs.CR.bits.PLL1ON)
			{
				qemu_log_mask(LOG_GUEST_ERROR, "Cannot change PLL1 configuration while it is enabled!");
			}
			else
			{
				s->regs.raw[index] = value;
			}
			break;
        case RI_PLL2CFGR:
		case RI_PLL2DIVR:
		case RI_PLL2FRACR:
			if (s->regs.CR.bits.PLL2ON)
			{
				qemu_log_mask(LOG_GUEST_ERROR, "Cannot change PLL2 configuration while it is enabled!");
			}
			else
			{
				s->regs.raw[index] = value;
			}
			break;
            break;
		// case RI_CIER ... RI_CICR:
        case RI_CCIPR1:
			{
            	const REGDEF_NAME(h503_rcc,ccipr1) new = { .raw = value };
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_USART1], new.bits.USART1SEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_USART2], new.bits.USART2SEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_USART3], new.bits.USART3SEL);
				s->regs.CCIPR1.raw = new.raw;
			}
            break;
		case RI_CCIPR3:
			{
				const REGDEF_NAME(h503_rcc,ccipr3) new = { .raw = value };
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_SPI1], new.bits.SPI1SEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_SPI2], new.bits.SPI2SEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_SPI3], new.bits.SPI3SEL);
				s->regs.CCIPR3.raw = new.raw;
			}
			break;
		case RI_CCIPR4:
			{
				const REGDEF_NAME(h503_rcc,ccipr4) new = { .raw = value };
				clktree_set_selected_input(&s->SYSTICK, new.bits.SYSTICKSEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_USBHS], new.bits.USBSEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2C1], new.bits.I2C1SEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_I2C2], new.bits.I2C2SEL);
				// clktree_set_selected_input(&s->parent.pclocks[STM32_P_I3C1], new.bits.I3C1SEL);
				// clktree_set_selected_input(&s->parent.pclocks[STM32_P_I3C2], new.bits.I3C2SEL);
				s->regs.CCIPR4.raw = new.raw;
			}
			break;
		case RI_CCIPR5:
			{
				const REGDEF_NAME(h503_rcc,ccipr5) new = { .raw = value };
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_ADC1], new.bits.ADCDACSEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_DAC], new.bits.ADCDACSEL);
				clktree_set_selected_input(&s->parent.pclocks[STM32_P_RNG], new.bits.RNGSEL);
				clktree_set_selected_input(&s->PER_CK, new.bits.CKERPSEL);
				s->regs.CCIPR5.raw = new.raw;
			}
			break;
		case RI_AHB1RSTR ... RI_APB3RSTR:
			stm32_common_rcc_reset_write(&s->parent,value, PERIPH_BLOCKS[index - RI_AHB1RSTR]);
			s->regs.raw[index] = value;
			break;
		case RI_AHB1ENR ... RI_APB2ENR:
            stm32_common_rcc_enable_write(&s->parent, value, PERIPH_BLOCKS[index - RI_AHB1ENR]);
			s->regs.raw[index] = value;
            break;
		//case RI_AHB1LPENR ... RI_APB3LPENR:
		//	s->regs.raw[index] = value;
			// break;
        case RI_BDCR:
            stm32_rcc_RCC_BDCR_write(s, value, is_reset);
            break;
		case RI_RSR:
			{
				const REGDEF_NAME(h503_rcc,rsr) new = { .raw = value };
				if (new.bits.RMVF)
				{
					s->regs.RSR.raw = 0;
				}
			}
			break;
        default:
            WARN_UNIMPLEMENTED_REG(offset, write);
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
            WARN_UNIMPLEMENTED_REG(offset, read);
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
            WARN_UNIMPLEMENTED_REG(offset,write);
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
    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (stm32_h503_rcc_reginfo[i].not_reserved)
			stm32_rcc_writew(dev, i<<2U, stm32_h503_rcc_reginfo[i].reset_val, true);
	}
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    STM32H503_STRUCT_NAME(Rcc) *s = (STM32H503_STRUCT_NAME(Rcc) *)opaque;

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
    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(dev);

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

	// Parent takes care of HSI (64MHz), HSE, LSI (32K), LSE(32.768KHz)

	// Primary clock sources:
	clktree_create_src_clk(&s->DUMMY, "DUMMY", 0, false);
	clktree_create_src_clk(&s->HSI48_KER_CK, "HSI48_RC", 48*MHz, false);
	clktree_create_src_clk(&s->CSI_CK, "CSI_RC", 4*MHz, false);

	clktree_create_clk(&s->SYS_CK, "SYSCLK", 1, 1, true, 250*MHz, SW_HSISYS,
						&s->parent.HSICLK, &s->CSI_CK, &s->parent.HSECLK, &s->PLL1PCLK, NULL);

	clktree_create_clk(&s->HSI_KER_CK, "HSI_KER_CK", 1, 1, true, 64*MHz, 0, &s->parent.HSICLK, NULL);

    clktree_create_clk(&s->HCLK, "HCLK", 1, 1, true, 250*MHz, 0, &s->SYS_CK, NULL);
    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);
    clktree_create_clk(&s->HCLK8, "HCLK8", 1, 8, true, 8*MHz, 0, &s->HCLK, NULL);

	clktree_create_clk(&s->HCLK1, "HCLK1", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
	clktree_create_clk(&s->HCLK2, "HCLK2", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PLL1CLK,  "PLL1CLK", 1, 2, true, 48*MHz, CLKTREE_NO_INPUT, &s->DUMMY, &s->parent.HSICLK, &s->CSI_CK, &s->parent.HSECLK,  NULL);
	clktree_create_clk(&s->PLL1PCLK, "PLL1P", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL1CLK, NULL);
	clktree_create_clk(&s->PLL1QCLK, "PLL1Q", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL1CLK, NULL);
	clktree_create_clk(&s->PLL1RCLK, "PLL1R", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL1CLK, NULL);

	clktree_create_clk(&s->PLL2CLK,  "PLL2CLK", 1, 2, true, 48*MHz, CLKTREE_NO_INPUT, &s->DUMMY, &s->parent.HSICLK, &s->CSI_CK, &s->parent.HSECLK,  NULL);
	clktree_create_clk(&s->PLL2PCLK, "PLL2P", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL2CLK, NULL);
	clktree_create_clk(&s->PLL2QCLK, "PLL2Q", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL2CLK, NULL);
	clktree_create_clk(&s->PLL2RCLK, "PLL2R", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PLL2CLK, NULL);

	// input order needs checking
	clktree_create_clk(&s->SYSTICK, "SYSTICK", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK8, &s->LSI_KER_CK, &s->LSE_CK, NULL);

	clktree_create_clk(&s->PER_CK, "PER_CK", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->HSI_KER_CK, &s->CSI_CK, &s->HSE_CK, &s->DUMMY, NULL);

	// APB clocks:
	clktree_create_clk(&s->PCLK1, "PCLK1", 1, 1, true, 250*MHz, 0, &s->HCLK, NULL);
	clktree_create_clk(&s->PCLK2, "PCLK2", 1, 1, true, 250*MHz, 0, &s->HCLK, NULL);
	clktree_create_clk(&s->PCLK3, "PCLK3", 1, 1, true, 250*MHz, 0, &s->HCLK, NULL);

	// Timer doublers:
	clktree_create_clk(&s->TIM23PRE, "TIM23CLK", 2, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
	clktree_create_clk(&s->TIM1PRE, "TIM1CLK", 2, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);


	clktree_create_clk(&s->RTCHSEPRE, "RTCHSEPRE", 1, 1, true, 1U*MHz, 0, &s->HSE_CK, NULL);

	// IWDG comes directly from LSI:
	INIT_PCLK_NSM(IWDG, 0, &s->parent.LSICLK, NULL);

	// RTC has choices:
	INIT_PCLK_NSM(RTC, CLKTREE_NO_INPUT, &s->DUMMY, &s->LSE_CK, &s->LSI_KER_CK, &s->RTCHSEPRE, NULL);

	// PWR is directly off of SYSCLK:
	INIT_PCLK_NSM(PWR, 0, &s->SYS_CK, NULL);


    /* AHB Peripheral clocks */
	// AHB1
	INIT_PCLK_NSM(DMA1, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(DMA2, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(CRC, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(FSMC, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(BKP, 0, &s->HCLK, NULL);

	// AHB2 - GPIOs, analog, RNG and HASH:
	INIT_PCLK_NSM(GPIOA, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(GPIOB, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(GPIOC, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(GPIOD, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(GPIOH, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(ADC1, 0, &s->HCLK, &s->SYS_CK, &s->PLL2RCLK, &s->HSE_CK, &s->HSI_KER_CK, &s->CSI_CK, NULL);
	INIT_PCLK_NSM(DAC,  0, &s->HCLK, &s->SYS_CK, &s->PLL2RCLK, &s->HSE_CK, &s->HSI_KER_CK, &s->CSI_CK, NULL);
	INIT_PCLK_NSM(HASH, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(RNG, 0, &s->HSI48_KER_CK, &s->PLL1QCLK, &s->LSE_CK, &s->LSI_KER_CK, NULL);

	// AHB3
	INIT_PCLK_NSM(RCC, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(PWR, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(EXTI, 0, &s->HCLK, NULL);
	INIT_PCLK_NSM(DBG, 0, &s->HCLK, NULL);

	// Everything else is APB

	// APB1
	INIT_PCLK_NSM(TIM2, 0, &s->TIM23PRE, NULL);
	INIT_PCLK_NSM(TIM3, 0, &s->TIM23PRE, NULL);
	INIT_PCLK_NSM(TIM6, 0, &s->PCLK1, NULL);
	INIT_PCLK_NSM(TIM7, 0, &s->PCLK1, NULL);
	INIT_PCLK_NSM(WWDG, 0, &s->PCLK1, NULL);
	// opamp?
	INIT_PCLK_NSM(SPI2, 0, &s->PLL1QCLK, &s->PLL2PCLK, &s->AUDIOCLK, &s->PCLK1, NULL);
	INIT_PCLK_NSM(SPI3, 0, &s->PLL1QCLK, &s->PLL2PCLK, &s->AUDIOCLK, &s->PCLK1, NULL);
	// Comp?
	INIT_PCLK_NSM(USART2, 0, &s->PCLK1, &s->PLL1QCLK, &s->HSI_KER_CK, &s->CSI_CK, &s->LSI_KER_CK, NULL);
	INIT_PCLK_NSM(USART3, 0, &s->PCLK1, &s->PLL1QCLK, &s->HSI_KER_CK, &s->CSI_CK, &s->LSI_KER_CK, NULL);
	INIT_PCLK_NSM(I2C1, 0, &s->PCLK1, &s->PLL2RCLK, &s->HSI_KER_CK, &s->CSI_CK, NULL);
	INIT_PCLK_NSM(I2C2, 0, &s->PCLK1, &s->PLL2RCLK, &s->HSI_KER_CK, &s->CSI_CK, NULL);
	// I3C1, CRS, DTS, LPTIM2, FDCAN1

	// APB2:
	INIT_PCLK_NSM(TIM1, 0, &s->TIM1PRE, NULL);
	INIT_PCLK_NSM(SPI1, 0, &s->PLL1QCLK, &s->PLL2PCLK, &s->AUDIOCLK, &s->PCLK2, NULL);
	INIT_PCLK_NSM(USART1, 0, &s->PCLK1, &s->PLL1QCLK, &s->HSI_KER_CK, &s->CSI_CK, &s->LSI_KER_CK, NULL);
	INIT_PCLK_NSM(USBHS, 0, &s->DUMMY, &s->PLL1QCLK, &s->PLL2QCLK, &s->HSI48_KER_CK, NULL);

	// APB3: SBS, LPUART1, I3C2, LPTIM1, RTC, TAMP
	INIT_PCLK_NSM(LPTIM1, 0, &s->PCLK3, NULL);


// vf'd end

}



static void stm32_rcc_init(Object *obj)
{

    STM32H503_STRUCT_NAME(Rcc) *s = STM32H503_RCC(obj);

    STM32_MR_IO_INIT(&s->parent.iomem, obj, &stm32_rcc_ops, s, 1U * KiB);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->parent.iomem);
}

static const VMStateDescription vmstate_stm32h503_rcc = {
    .name = TYPE_STM32H503_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(parent.pclocks, STM32H503_STRUCT_NAME(Rcc), STM32_P_COUNT, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(SYS_CK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(PLLPCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(PLLQCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(PLLRCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(PLLCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(HCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        // VMSTATE_STRUCT(PCLK,STM32H503_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
		VMSTATE_UINT32_ARRAY(regs.raw, STM32H503_STRUCT_NAME(Rcc), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_rcc_reset);
    dc->realize = stm32_rcc_realize;
    dc->vmsd = &vmstate_stm32h503_rcc;
}

static TypeInfo stm32_rcc_info = {
    .name  = TYPE_STM32H503_RCC,
    .parent = TYPE_STM32COM_RCC,
    .instance_size  = sizeof(STM32H503_STRUCT_NAME(Rcc)),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init,
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
