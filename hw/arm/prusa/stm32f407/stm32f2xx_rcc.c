/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
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

#include "stm32f2xx_rcc.h"
#include "qemu-common.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include <stdio.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "../utility/macros.h"

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

enum pll_src
{
	PLL_HSI,
	PLL_HSE
};

enum sw_src
{
	SW_HSI,
	SW_HSE,
	SW_PLL,
};

/* HELPER FUNCTIONS */

static const uint8_t AHB1_PERIPHS[32] = {
    STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, STM32_P_GPIOE, STM32_P_GPIOF, STM32_P_GPIOG, STM32_P_GPIOH,
    STM32_P_GPIOI, 0, 0, 0, STM32_P_CRC, 0, 0, 0,
    0, 0, 0, 0, 0, 0, STM32_P_DMA1, STM32_P_DMA2,
    0, 0, 0/*eth*/, 0, 0, STM32_P_USB, 0, 0
};

static const uint8_t AHB2_PERIPHS[32] = {
    STM32_P_DCMI, 0, 0, 0, STM32_P_CRYP, STM32_P_HASH, STM32_P_RNG, STM32_P_USB2,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t AHB3_PERIPHS[32] = {
    STM32_P_FSMC, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB1_PERIPHS[32] = {
    STM32_P_TIM2, STM32_P_TIM3, STM32_P_TIM4, STM32_P_TIM5, STM32_P_TIM6, STM32_P_TIM7, STM32_P_TIM12, STM32_P_TIM13,
    STM32_P_TIM14, 0, 0, STM32_P_WWDG, 0, 0, STM32_P_SPI2, STM32_P_SPI3,
    0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, STM32_P_UART5, STM32_P_I2C1, STM32_P_I2C2, STM32_P_I2C3,
    0, STM32_P_CAN1, STM32_P_CAN2, 0, STM32_P_PWR, STM32_P_DAC, 0, 0
};

static const uint8_t APB2RST_PERIPHS[32] = {
    STM32_P_TIM1, STM32_P_TIM8, 0, 0, STM32_P_UART1, STM32_P_UART6, 0, 0,
    STM32_P_ADC_ALL, 0, 0, STM32_P_SDIO, STM32_P_SPI1, 0, STM32_P_SYSCFG, 0,
    STM32_P_TIM9, STM32_P_TIM10, STM32_P_TIM11, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t APB2EN_PERIPHS[32] = {
    STM32_P_TIM1, STM32_P_TIM8, 0, 0, STM32_P_UART1, STM32_P_UART6, 0, 0,
    STM32_P_ADC1, STM32_P_ADC2, STM32_P_ADC3, STM32_P_SDIO, STM32_P_SPI1, 0, STM32_P_SYSCFG, 0,
    STM32_P_TIM9, STM32_P_TIM10, STM32_P_TIM11, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t (*RESET_BLOCKS[6])[32] = {
	&AHB1_PERIPHS,
	&AHB2_PERIPHS,
	&AHB3_PERIPHS,
	NULL,
	&APB1_PERIPHS,
	&APB2RST_PERIPHS
};

static const uint8_t (*ENA_BLOCKS[6])[32] = {
	&AHB1_PERIPHS,
	&AHB2_PERIPHS,
	&AHB3_PERIPHS,
	NULL,
	&APB1_PERIPHS,
	&APB2EN_PERIPHS
};

enum reg_index {
	RI_CR,
	RI_PLLCFGR,
	RI_CFGR,
	RI_CIR,
	RI_AHB1RSTR,
	RI_AHB2RSTR,
	RI_AHB3RSTR,
	RI_APB1RSTR = 0x20/4,
	RI_APB2RSTR,
	RI_AHB1ENR = 0x30/4,
	RI_AHB2ENR,
	RI_AHB3ENR,
	RI_APB1ENR = 0x40/4,
	RI_APB2ENR,
	RI_AHB1LPENR = 0x50/4,
	RI_AHB2LPENR,
	RI_AHB3LPENR,
	RI_APB1LPENR = 0x60/4,
	RI_APB2LPENR,
	RI_BDCR = 0x70/4,
	RI_CSR,
	RI_SSCGR = 0x80/4,
	RI_PLLI2SCFGR,
	RI_END
};

static const stm32_reginfo_t stm32f4xx_rcc_reginfo[RI_END] =
{
	[RI_CR] = {.mask = 0x0F0FFFFB, .reset_val = 0x83},

	[RI_PLLCFGR] = {.mask = 0x7FFF, .reset_val = 0x24003010, .unimp_mask = 0x7FFF}, //
	[RI_CFGR] = {.mask = 0xFFFF7F3F, .unimp_mask = 0xFFFF0000}, //
	[RI_CIR] = {.mask = 0xFF3F7F73, .reset_val =  0x1000, .unimp_mask = UINT32_MAX}, //
	[RI_AHB1RSTR] = {.mask = 0x226011FF},
	[RI_AHB2RSTR] = {.mask = 0x000000F1},
	[RI_AHB3RSTR] = {.mask = 0x00000001},
	[RI_AHB3RSTR+1U] = {.is_reserved = true},
	[RI_APB1RSTR] = {.mask = 0x36FEC9FF},
	[RI_APB2RSTR] = {.mask = 0x00075933},
	[(RI_APB2RSTR + 1U) ... (RI_APB2RSTR + 2U)] = {.is_reserved = true},
	[RI_AHB1ENR] = {.mask = 0x7E7411FF, .unimp_mask = ~0x226011FF},
	[RI_AHB2ENR] = {.mask = 0x000000F1},
	[RI_AHB3ENR] = {.mask = 0x00000001},
	[RI_AHB3ENR+1U] = {.is_reserved = true},
	[RI_APB1ENR] = {.mask = 0x36FEC9FF},
	[RI_APB2ENR] = {.mask = 0x00075F33},
	[(RI_APB2ENR + 1U) ... (RI_APB2ENR + 2U)] = {.is_reserved = true},
	[RI_AHB1LPENR] = {.mask = 0x7E7411FF, .reset_val = 0x7E6791FF, .unimp_mask = UINT32_MAX},
	[RI_AHB2LPENR] = {.mask = 0x000000F1, .reset_val = 0xF1, .unimp_mask = UINT32_MAX},
	[RI_AHB3LPENR] = {.mask = 0x00000001, .reset_val = 0x01, .unimp_mask = UINT32_MAX},
	[RI_AHB3LPENR+1U] = {.is_reserved = true},
	[RI_APB1LPENR] = {.mask = 0x36FEC9FF, .reset_val = 0x36FEC9FF, .unimp_mask = UINT32_MAX},
	[RI_APB2LPENR] = {.mask = 0x00075F33, .reset_val = 0x00075F33, .unimp_mask = UINT32_MAX},
	[(RI_APB2LPENR + 1U) ... (RI_APB2LPENR + 2U)] = {.is_reserved = true},
	[RI_BDCR] = {.mask = 0x17D801, .unimp_mask = 0x17D801},
	[RI_CSR] = { .mask = 0xFF000003, .reset_val = 0x0E000000},
	[(RI_CSR + 1U) ... (RI_SSCGR - 1U)] = {.is_reserved = true},
	[RI_SSCGR] = {.unimp_mask = UINT32_MAX, .mask = 0xCFFFFFFF},
	[RI_PLLI2SCFGR] = {.mask = 0x70007FC0, .reset_val = 0x20003000 },
};


QEMU_BUILD_BUG_MSG(RI_END != R_RCC_MAX, "maxima definitions for F2xx RCC Misaligned!");
/* REGISTER IMPLEMENTATION */

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    REGDEF_NAME(rcc,cr) cr = {.raw = new_value};

    if((clktree_is_enabled(&s->PLLCLK) && !cr.PLLON) &&
       s->regs.CFGR.SW == SW_PLL) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(&s->PLLCLK, cr.PLLON);

    if((clktree_is_enabled(&s->parent.HSECLK) && !cr.HSEON) &&
       (s->regs.CFGR.SW == SW_HSE || s->regs.CFGR.SW == SW_PLL)
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSECLK, cr.HSEON);

    if((clktree_is_enabled(&s->parent.HSECLK) && !cr.HSION) &&
       (s->regs.CFGR.SW == SW_HSI || s->regs.CFGR.SW == SW_PLL)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->parent.HSICLK, cr.HSION);

    clktree_set_enabled(&s->PLLI2SCLK, cr.PLLI2SON);

}

static void stm32_rcc_RCC_PLLCFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PLLM division factor */
    REGDEF_NAME(rcc, pllcfgr) r = {.raw = new_value};
    if (r.PLLM <= 1) {
        qemu_log_mask(LOG_GUEST_ERROR,"PLLM division factor cannot be 0 or 1. Given: %u", r.PLLM);
    }

    /* PLLN multiplication factor */
    if (r.PLLN <= 1 || r.PLLN >= 433) {
        qemu_log_mask(LOG_GUEST_ERROR,"PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", r.PLLN);
    }

    /* PPLP division factor */
    const uint8_t new_PLLP = 2 + (2 * r.PLLP);

    /* Warn in case of illegal writes: */
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(&s->PLLCLK) /* && TODO: !clktree_is_enabled(s->PLLI2SCLK) */);
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled";
            if (r.PLLM != s->regs.PLLCFGR.PLLM) {
                qemu_log_mask(LOG_GUEST_ERROR,warning_fmt, "PLLM");
            }
            if (r.PLLM != s->regs.PLLCFGR.PLLN) {
                qemu_log_mask(LOG_GUEST_ERROR,warning_fmt, "PLLN");
            }
            if (r.PLLSRC != s->regs.PLLCFGR.PLLSRC) {
                qemu_log_mask(LOG_GUEST_ERROR,warning_fmt, "PLLSRC");
            }
        }
    }

	s->regs.PLLCFGR.raw = r.raw;
    clktree_set_scale(&s->PLLM, r.PLLN, r.PLLM);

    clktree_set_selected_input(&s->PLLM, r.PLLSRC);

    clktree_set_scale(&s->PLLCLK, 1, new_PLLP);
}

static void stm32_rcc_RCC_PLLI2SCFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    REGDEF_NAME(rcc, plli2scfgr) r = {.raw = new_value};
    /* PLLR division factor */
    if (r.PLLI2SR < 2 || r.PLLI2SR > 7) {
         qemu_log_mask(LOG_GUEST_ERROR,"PLLR multiplication factor must be between 2 and 7. Given: %u", r.PLLI2SR);
    }

    // /* PLLQ division factor */
    // const uint16_t new_PLLQ = (new_value & RCC_PLLI2SCFGR_PLLQ_MASK) >> RCC_PLLI2SCFGR_PLLQ_START;
    // if (new_PLLQ > 15) {
    //     qemu_log_mask(LOG_GUEST_ERROR,"PLLQ multiplication factor must be between 0 and 15 "
    //              "(inclusive). Given: %u", r.);
    // }

    /* PLLN multiplication factor */
    if (r.PLLN < 2 || r.PLLN > 433) {
        qemu_log_mask(LOG_GUEST_ERROR,"PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", r.PLLN);
    }


    /* Warn in case of illegal writes: */
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(&s->PLLI2SCLK) /* && TODO: !clktree_is_enabled(s->PLLI2SCLK) */);
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled";
            if (r.PLLI2SR != s->regs.PLLI2SCFGR.PLLI2SR) {
                qemu_log_mask(LOG_GUEST_ERROR,warning_fmt, "PLLR");
            }
            // if (new_PLLQ != s->RCC_PLLI2SCFGR_PLLQ) {
            //     printf(warning_fmt, "PLLQ");
            // }
            if (r.PLLN != s->regs.PLLI2SCFGR.PLLN) {
                qemu_log_mask(LOG_GUEST_ERROR,warning_fmt, "PLLN");
            }
        }
    }

	s->regs.PLLI2SCFGR.raw = r.raw;
    clktree_set_scale(&s->PLLI2SM, r.PLLN, s->regs.PLLCFGR.PLLM);
    clktree_set_scale(&s->PLLI2SCLK, 1, r.PLLI2SR);
}

static void stm32_rcc_RCC_CFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PPRE2 */
	REGDEF_NAME(rcc, cfgr) cfgr = {.raw = new_value};
    if(cfgr.PPRE2 < 0x4) {
        clktree_set_scale(&s->PCLK2, 1, 1);
        clktree_set_scale(&s->TIMCLK2, 1, 1);
    } else {
        clktree_set_scale(&s->PCLK2, 1, 2 * (cfgr.PPRE2 - 3));
		clktree_set_scale(&s->TIMCLK2, 2, 1);
    }

    /* PPRE1 */
    if(cfgr.PPRE1 < 4) {
        clktree_set_scale(&s->PCLK1, 1, 1);
		clktree_set_scale(&s->TIMCLK1, 1, 1);
    } else {
        clktree_set_scale(&s->PCLK1, 1, 2 * (cfgr.PPRE1 - 3));
		clktree_set_scale(&s->TIMCLK1, 2, 1);
    }

    /* HPRE */
    if(cfgr.HPRE < 8) {
        clktree_set_scale(&s->HCLK, 1, 1);
    } else {
        clktree_set_scale(&s->HCLK, 1, 2 * (cfgr.HPRE - 7));
    }

    /* SW */
    switch(cfgr.SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(&s->SYSCLK, cfgr.SW);
			cfgr.SWS = cfgr.SW;
            break;
        default:
            printf("Invalid input selected for SYSCLK");
            break;
    }
	if (cfgr.RTCPRE<2)
		clktree_set_enabled(&s->RTCCLK,false);
	else
		clktree_set_scale(&s->RTCCLK, 1, cfgr.RTCPRE);

	s->regs.CFGR.raw = cfgr.raw;
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;
	uint32_t index = offset>>2U;
	uint32_t data = 0;
	CHECK_BOUNDS_R(index, RI_END, stm32f4xx_rcc_reginfo, "F4xx RCC");
    switch (index) {
        case RI_CR:
		{
            REGDEF_NAME(rcc,cr) cr;
			cr.HSEON = cr.HSERDY = clktree_is_enabled(&s->parent.HSECLK);
			cr.HSION = cr.HSIRDY = clktree_is_enabled(&s->parent.HSICLK);
			cr.PLLON = cr.PLLRDY = clktree_is_enabled(&s->PLLCLK);
			cr.PLLI2SON = cr.PLLI2SRDY = clktree_is_enabled(&s->PLLI2SCLK);
			data = cr.raw;
		}
			break;
        case RI_CFGR:
        case RI_AHB1ENR ... RI_AHB3ENR:
		case RI_APB1ENR ... RI_APB2ENR:
            data = s->regs.raw[index];
			break;
        case RI_BDCR:
		{
            REGDEF_NAME(rcc,bdcr) bdcr;
			bdcr.LSEON = bdcr.LSERDY = clktree_is_enabled(&s->parent.LSECLK);
			bdcr.RTCEN = clktree_is_enabled(&s->parent.pclocks[STM32_P_RTC]);
			data = bdcr.raw;
			break;
		}
        case RI_CSR:
		{
			REGDEF_NAME(rcc, csr) csr;
			csr.LSION = csr.LSIRDY = clktree_is_enabled(&s->parent.LSICLK);
			data = csr.raw;
			break;
		}
        case RI_CIR:
        case RI_PLLCFGR:
        case RI_PLLI2SCFGR:
            data = s->regs.raw[index];
			break;
        case RI_AHB1RSTR ... RI_AHB3RSTR: // just return zero since the reset takes no time in sim
		case RI_APB1RSTR ... RI_APB2RSTR:
        default:
			break;
    }
	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, offset&0x3, 0b100);
	return data;

}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t data, unsigned size)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;
	uint32_t index = offset>>2U;
	CHECK_BOUNDS_W(index, data, RI_END, stm32f4xx_rcc_reginfo, "F2xx RCC");
	uint32_t oldval = stm32_rcc_read(opaque, index<<2U,4);
	ADJUST_FOR_OFFSET_AND_SIZE_W(oldval, data, size, (offset&0x3), 0b101);
	CHECK_UNIMP_RESVD(data, stm32f4xx_rcc_reginfo, index);
    switch(index) {
        case RI_CR:
            stm32_rcc_RCC_CR_write(s, data, false);
            break;
        case RI_PLLCFGR:
            stm32_rcc_RCC_PLLCFGR_write(s, data, false);
            break;
        case RI_CFGR:
            stm32_rcc_RCC_CFGR_write(s, data, false);
            break;
        case RI_AHB1RSTR ... RI_AHB3RSTR:
		case RI_APB1RSTR ... RI_APB2RSTR:
			stm32_common_rcc_reset_write(&s->parent, data, RESET_BLOCKS[index-RI_AHB1RSTR]);
			s->regs.raw[index] = data;
            break;
        case RI_AHB1ENR ... RI_AHB3ENR:
		case RI_APB1ENR ... RI_APB2ENR:
			stm32_common_rcc_enable_write(&s->parent, data, ENA_BLOCKS[index-RI_AHB1ENR]);
			s->regs.raw[index] = data;
            break;
        case RI_BDCR:
		{
			REGDEF_NAME(rcc, bdcr) bdcr = {.raw = data};
            clktree_set_enabled(&s->parent.LSECLK, bdcr.LSEON);
			clktree_set_selected_input(&s->parent.pclocks[STM32_P_RTC], bdcr.RTC_SEL -1U );
			clktree_set_enabled(&s->parent.pclocks[STM32_P_RTC], bdcr.RTCEN);
            break;
		}
        case RI_CSR:
		{
			REGDEF_NAME(rcc, csr) r = {.raw = data};
   			clktree_set_enabled(&s->parent.LSICLK, r.LSION);
		}
            break;
        case RI_PLLI2SCFGR:
            stm32_rcc_RCC_PLLI2SCFGR_write(s, data, false);
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
    Stm32f2xxRcc *s = STM32F2xx_RCC(dev);

	for (int i=0; i<RI_END; i++)
		stm32_rcc_write(s, i<<2U, stm32f4xx_rcc_reginfo[i].reset_val,4);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    uint32_t hclk_freq = 0;
    // uint32_t ext_ref_freq = 0;

    hclk_freq = clktree_get_output_freq(&s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if (hclk_freq > 0) {
        // ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = 1000000000LL / hclk_freq;
    }
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_realize(DeviceState *dev, Error **errp)
{
    Stm32f2xxRcc *s = STM32F2xx_RCC(dev);
	s->parent.realize_func(dev, errp);
    s->hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);


    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */

    clktree_create_clk(&s->PLLM, "PLLM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, &s->parent.HSICLK,
                                 &s->parent.HSECLK, NULL);
    clktree_create_clk(&s->PLLCLK, "PLLCLK", 1, 2, false, 120000000, 0, &s->PLLM, NULL);
    clktree_create_clk(&s->PLL48CLK, "PLL48CLK", 1, 1, false, 48000000, 0, &s->PLLM, NULL);

    clktree_create_clk(&s->PLLI2SM, "PLLI2SM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, &s->PLLM, NULL);
    clktree_create_clk(&s->PLLI2SCLK, "PLLI2SCLK", 1, 2, false, 120000000, 0, &s->PLLI2SM, NULL);

    clktree_create_clk(&s->SYSCLK, "SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   &s->parent.HSICLK, &s->parent.HSECLK, &s->PLLCLK, NULL);

	clktree_create_clk(&s->RTCCLK, "RTCHSE", 1, 1, false, CLKTREE_NO_MAX_FREQ,
                                   0, &s->parent.HSECLK, NULL);


    // HCLK: to AHB bus, core memory and DMA
    clktree_create_clk(&s->HCLK, "HCLK", 1, 1, true, 168000000, 0, &s->SYSCLK, NULL);
    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);

    // Clock source for APB1 peripherals:

    clktree_create_clk(&s->PCLK1, "PCLK1", 1, 1, true, 30000000, 0, &s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    clktree_create_clk(&s->PCLK2, "PCLK2", 1, 1, true, 60000000, 0, &s->HCLK, NULL);

	clktree_create_clk(&s->TIMCLK1, "TIMCLK1", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
	clktree_create_clk(&s->TIMCLK2, "TIMCLK2", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);

    /* Peripheral clocks */
    INIT_PCLK_NSM(GPIOA, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOB, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOC, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOD, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOE, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOF, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOG, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOH, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOI, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOJ, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(GPIOK, 0, &s->HCLK, NULL);

    INIT_PCLK_NSM(CRC, 0, &s->HCLK, NULL);

    INIT_PCLK_NSM(DMA1, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(DMA2, 0, &s->HCLK, NULL);

    INIT_PCLK_NSM(SYSCFG, 0, &s->PCLK2, NULL);

    INIT_PCLK_NSM(UART1, 0, &s->PCLK2, NULL);
    INIT_PCLK_NSM(UART2, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART3, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART4, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART5, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART6, 0, &s->PCLK2, NULL);
    INIT_PCLK_NSM(UART7, 0, &s->PCLK1, NULL);
    INIT_PCLK_NSM(UART8, 0, &s->PCLK1, NULL);


    INIT_PCLK_NSM(ADC1, 0, &s->PCLK2, NULL);
    INIT_PCLK_NSM(ADC2, 0, &s->PCLK2, NULL);
    INIT_PCLK_NSM(ADC3, 0, &s->PCLK2, NULL);

    // Timers run at 2x the APB speed
    INIT_PCLK_NSM(TIM1, 	0, &s->TIMCLK2, NULL);
    INIT_PCLK_NSM(TIM2, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM3, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM4, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM5, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM6, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM7, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM8, 	0, &s->TIMCLK2, NULL);
    INIT_PCLK_NSM(TIM9, 	0, &s->TIMCLK2, NULL);
    INIT_PCLK_NSM(TIM10, 	0, &s->TIMCLK2, NULL);
    INIT_PCLK_NSM(TIM11, 	0, &s->TIMCLK2, NULL);
    INIT_PCLK_NSM(TIM12, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM13, 	0, &s->TIMCLK1, NULL);
    INIT_PCLK_NSM(TIM14, 	0, &s->TIMCLK1, NULL);


    INIT_PCLK_NSM(IWDG, 0, &s->parent.LSICLK, NULL);

    INIT_PCLK_NSM(DCMI, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(CRYP, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(HASH, 0, &s->HCLK, NULL);
    INIT_PCLK_NSM(RNG, 0, &s->HCLK, NULL);

    INIT_PCLK(FSMC, 1, 2, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

	INIT_PCLK_NSM(RTC, CLKTREE_NO_INPUT, &s->parent.LSECLK, &s->parent.LSICLK, &s->RTCCLK, NULL);
}


static void stm32_rcc_init(Object *obj)
{

    Stm32f2xxRcc *s = STM32F2xx_RCC(obj);

    memory_region_init_io(&s->parent.iomem, obj, &stm32_rcc_ops, s,
                          "rcc", 1U*KiB);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->parent.iomem);
}

static const VMStateDescription vmstate_STM32F2xx_RCC = {
    .name = TYPE_STM32F2xx_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(SYSCLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(RTCCLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLM,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLCLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLL48CLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLI2SM,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLI2SCLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HCLK,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK1,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK2,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(TIMCLK1,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(TIMCLK2,Stm32f2xxRcc, 1, vmstate_stm32_common_rcc_clk, Clk_t),
		VMSTATE_UINT32_ARRAY(regs.raw, Stm32f2xxRcc, RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_rcc_reset;
    dc->realize = stm32_rcc_realize;
    dc->vmsd = &vmstate_STM32F2xx_RCC;
}

static TypeInfo stm32_rcc_info = {
    .name  = TYPE_STM32F2xx_RCC,
    .parent = TYPE_STM32COM_RCC,
    .instance_size  = sizeof(Stm32f2xxRcc),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init,
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
