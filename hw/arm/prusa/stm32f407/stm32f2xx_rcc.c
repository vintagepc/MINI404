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
// #define STATE_DEBUG_VAR rcc_dbg
#include "../utility/macros.h"

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

#define HSI_FREQ 16000000
#define LSI_FREQ 32000

#define RCC_CR_RESET_VALUE      0x00000083
#define RCC_CR_OFFSET           0x00
#define RCC_CR_PLLI2SRDY_BIT    27
#define RCC_CR_PLLI2SON_BIT     26
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_CSSON_BIT        19
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x0000ff00
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0

#define RCC_PLLCFGR_RESET_VALUE  0x24003010
#define RCC_PLLCFGR_OFFSET       0x04
#define RCC_PLLCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLCFGR_PLLQ_START   24
#define RCC_PLLCFGR_PLLSRC_BIT   22
#define RCC_PLLCFGR_PLLP_MASK    0x00030000
#define RCC_PLLCFGR_PLLP_START   16
#define RCC_PLLCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLCFGR_PLLN_START   6
#define RCC_PLLCFGR_PLLM_MASK    0x0000003F
#define RCC_PLLCFGR_PLLM_START   0

#define RCC_CFGR_RESET_VALUE     0x00000000
#define RCC_CFGR_OFFSET          0x08
#define RCC_CFGR_MCO2_START      30
#define RCC_CFGR_MCO2_MASK       0xC0000000
#define RCC_CFGR_MCO2PRE_START   27
#define RCC_CFGR_MCO2PRE_MASK    0x38000000
#define RCC_CFGR_MCO1PRE_START   24
#define RCC_CFGR_MCO1PRE_MASK    0x07000000
#define RCC_CFGR_I2CSRC_BIT      23
#define RCC_CFGR_MCO1_START      21
#define RCC_CFGR_MCO1_MASK       0x00600000
#define RCC_CFGR_RTCPRE_START    16
#define RCC_CFGR_RTCPRE_MASK     0x001F0000
#define RCC_CFGR_PPRE2_START     13
#define RCC_CFGR_PPRE2_MASK      0x0000E000
#define RCC_CFGR_PPRE1_START     10
#define RCC_CFGR_PPRE1_MASK      0x00001C00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000F0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000C
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

#define RCC_CIR_RESET_VALUE      0x00000000
#define RCC_CIR_OFFSET           0x0C
#define RCC_CIR_CSSC_BIT         23
#define RCC_CIR_PLLI2SRDYC_BIT   21
#define RCC_CIR_PLLRDYC_BIT      20
#define RCC_CIR_HSERDYC_BIT      19
#define RCC_CIR_HSIRDYC_BIT      18
#define RCC_CIR_LSERDYC_BIT      17
#define RCC_CIR_LSIRDYC_BIT      16
#define RCC_CIR_PLLI2SRDYIE_BIT  13
#define RCC_CIR_PLLRDYIE_BIT     12
#define RCC_CIR_HSERDYIE_BIT     11
#define RCC_CIR_HSIRDYIE_BIT     10
#define RCC_CIR_LSERDYIE_BIT     9
#define RCC_CIR_LSIRDYIE_BIT     8
#define RCC_CIR_CSSF_BIT         7
#define RCC_CIR_PLLI2SRDYF_BIT  5
#define RCC_CIR_PLLRDYF_BIT     4
#define RCC_CIR_HSERDYF_BIT     3
#define RCC_CIR_HSIRDYF_BIT     2
#define RCC_CIR_LSERDYF_BIT     1
#define RCC_CIR_LSIRDYF_BIT     0

static_assert(STM32_PERIPH_COUNT<=255,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

#define RCC_AHB1RSTR_OFFSET 0x10
static const uint8_t AHB1RST_PERIPHS[32] = {
    STM32_GPIOA, STM32_GPIOB, STM32_GPIOC, STM32_GPIOD, STM32_GPIOE, STM32_GPIOF, STM32_GPIOG, STM32_GPIOH,
    STM32_GPIOI, 0, 0, 0, STM32_CRC, 0, 0, 0, 
    0, 0, 0, 0, 0, STM32_DMA1, STM32_DMA2, 0,
    0, 0 /* TODO STM32_ETH*/, 0, 0, 0, STM32_USB, 0, 0
};


#define RCC_AHB2RSTR_OFFSET 0x14
static const uint8_t AHB2RST_PERIPHS[32] = {
    STM32_DCMI_PERIPH, 0, 0, 0, STM32_CRYP_PERIPH, STM32_HASH_PERIPH, STM32_RNG_PERIPH, 0 /*STM32_OTGFS*/,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

#define RCC_AHB3RSTR_OFFSET 0x18
static const uint8_t AHB3RST_PERIPHS[32] = {
    STM32_FSMC, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

#define RCC_APB1RSTR_OFFSET 0x20
static const uint8_t APB1RST_PERIPHS[32] = {
    STM32_TIM2, STM32_TIM3, STM32_TIM4, STM32_TIM5, STM32_TIM6, STM32_TIM7, STM32_TIM12, STM32_TIM13,
    STM32_TIM14, 0, 0, STM32_WWDG, 0, 0, STM32_SPI2, STM32_SPI3, 
    0, STM32_UART2, STM32_UART3, STM32_UART4, STM32_UART5, STM32_I2C1, STM32_I2C2, STM32_I2C3,
    0, STM32_CAN1, STM32_CAN2, 0, STM32_PWR, STM32_DAC, 0, 0
};

#define RCC_APB2RSTR_RESET_VALUE     0x00000000
#define RCC_APB2RSTR_OFFSET          0x24
static const uint8_t APB2RST_PERIPHS[32] = {
    STM32_TIM1, STM32_TIM8, 0, 0, STM32_UART1, STM32_UART6, 0, 0,
    STM32_ADC1, 0, 0, STM32_SDIO, STM32_SPI1, 0, STM32_SYSCFG, 0, 
    STM32_TIM9, STM32_TIM10, STM32_TIM11, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

#define RCC_AHB1ENR_RESET_VALUE      0x00000000
#define RCC_AHB1ENR_OFFSET           0x30
#define RCC_AHB1ENR_OTGHSULPIEN_BIT  30
#define RCC_AHB1ENR_OTGHSEN_BIT      29
#define RCC_AHB1ENR_ETHMACPTPEN_BIT  28
#define RCC_AHB1ENR_ETHMACRXEN_BIT   27
#define RCC_AHB1ENR_ETHMACTXEN_BIT   26
#define RCC_AHB1ENR_ETHMACEN_BIT     25
#define RCC_AHB1ENR_DMA2EN_BIT       22
#define RCC_AHB1ENR_DMA1EN_BIT       21
#define RCC_AHB1ENR_BKPSRAMEN_BIT    18
#define RCC_AHB1ENR_CRCEN_BIT        12
#define RCC_AHB1ENR_GPIOKEN_BIT      10
#define RCC_AHB1ENR_GPIOJEN_BIT      9
#define RCC_AHB1ENR_GPIOIEN_BIT      8
#define RCC_AHB1ENR_GPIOHEN_BIT      7
#define RCC_AHB1ENR_GPIOGEN_BIT      6
#define RCC_AHB1ENR_GPIOFEN_BIT      5
#define RCC_AHB1ENR_GPIOEEN_BIT      4
#define RCC_AHB1ENR_GPIODEN_BIT      3
#define RCC_AHB1ENR_GPIOCEN_BIT      2
#define RCC_AHB1ENR_GPIOBEN_BIT      1
#define RCC_AHB1ENR_GPIOAEN_BIT      0

#define RCC_AHB2ENR_OFFSET 0x34
#define RCC_AHB2ENR_DCMIEN_BIT       0
#define RCC_AHB2ENR_CRYPEN_BIT       4
#define RCC_AHB2ENR_HASHEN_BIT       5
#define RCC_AHB2ENR_RNGEN_BIT        6
#define RCC_AHB2ENR_OTGFSEN_BIT      7

#define RCC_AHB3ENR_OFFSET 0x38
#define RCC_AHB3ENR_FSMCEN_BIT       0

#define RCC_APB1ENR_RESET_VALUE  0x00000000
#define RCC_APB1ENR_OFFSET       0x40
#define RCC_APB1ENR_USART8EN_BIT 31
#define RCC_APB1ENR_USART7EN_BIT 30
#define RCC_APB1ENR_DACEN_BIT    29
#define RCC_APB1ENR_PWREN_BIT    28
#define RCC_APB1ENR_BKPEN_BIT    27
#define RCC_APB1ENR_CAN2EN_BIT   26
#define RCC_APB1ENR_CAN1EN_BIT   25
#define RCC_APB1ENR_CANEN_BIT    25
#define RCC_APB1ENR_USBEN_BIT    23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_USART5EN_BIT 20
#define RCC_APB1ENR_USART4EN_BIT 19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM14EN_BIT  8
#define RCC_APB1ENR_TIM13EN_BIT  7
#define RCC_APB1ENR_TIM12EN_BIT  6
#define RCC_APB1ENR_TIM7EN_BIT   5
#define RCC_APB1ENR_TIM6EN_BIT   4
#define RCC_APB1ENR_TIM5EN_BIT   3
#define RCC_APB1ENR_TIM4EN_BIT   2
#define RCC_APB1ENR_TIM3EN_BIT   1
#define RCC_APB1ENR_TIM2EN_BIT   0

#define RCC_APB2ENR_RESET_VALUE  0x00000000
#define RCC_APB2ENR_OFFSET       0x44
#define RCC_APB2ENR_MASK         0x00075F33
#define RCC_APB2ENR_TIM11EN_BIT      18
#define RCC_APB2ENR_TIM10EN_BIT      17
#define RCC_APB2ENR_TIM9EN_BIT       16
#define RCC_APB2ENR_SYSCFGEN_BIT 14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT   9
#define RCC_APB2ENR_ADC1EN_BIT   8
#define RCC_APB2ENR_USART6EN_BIT 5
#define RCC_APB2ENR_USART1EN_BIT 4
#define RCC_APB2ENR_TIM8EN_BIT   1
#define RCC_APB2ENR_TIM1EN_BIT   0

#define RCC_AHB1LPENR_RESET_VALUE  0x7E6791FF
#define RCC_AHB1LPENR_OFFSET       0x50

#define RCC_AHB2LPENR_RESET_VALUE  0x000000F1
#define RCC_AHB2LPENR_OFFSET       0x54

#define RCC_AHB3LPENR_RESET_VALUE  0x00000001
#define RCC_AHB3LPENR_OFFSET       0x58

#define RCC_APB1LPENR_RESET_VALUE  0x36FEC9FF
#define RCC_APB1LPENR_OFFSET       0x60

#define RCC_APB2LPENR_RESET_VALUE  0x00075F33
#define RCC_APB2LPENR_OFFSET       0x64

#define RCC_BDCR_RESET_VALUE       0x00000000
#define RCC_BDCR_OFFSET            0x70
#define RCC_BDCR_BDRST_BIT         16
#define RCC_BDCR_RTCEN_BIT         15
#define RCC_BDCR_RTCSEL_START      8
#define RCC_BDCR_RTCSEL_MASK       0x00000300
#define RCC_BDCR_LSEBYP_BIT        2
#define RCC_BDCR_LSERDY_BIT        1
#define RCC_BDCR_LSEON_BIT         0

#define RCC_CSR_RESET_VALUE        0x0E000000
#define RCC_CSR_OFFSET             0x74
#define RCC_CSR_LPWRRSTF_BIT       31
#define RCC_CSR_WWDGRSTF_BIT       30
#define RCC_CSR_IWDGRSTF_BIT       29
#define RCC_CSR_SFTRSTF_BIT        28
#define RCC_CSR_PORRSTF_BIT        27
#define RCC_CSR_PINRSTF_BIT        26
#define RCC_CSR_BORRSTF_BIT        25
#define RCC_CSR_RMVF_BIT           24
#define RCC_CSR_LSIRDY_BIT         1
#define RCC_CSR_LSION_BIT          0

#define RCC_SSCGR_RESET_VALUE      0x00000000
#define RCC_SSCGR_OFFSET           0x80

#define RCC_PLLI2SCFGR_RESET_VALUE 0x20003000
#define RCC_PLLI2SCFGR_OFFSET      0x84
#define RCC_PLLI2SCFGR_PLLR_MASK    0x70000000
#define RCC_PLLI2SCFGR_PLLR_START   28
#define RCC_PLLI2SCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLI2SCFGR_PLLQ_START   24
#define RCC_PLLI2SCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLI2SCFGR_PLLN_START   6



#define PLLSRC_HSI_SELECTED 0
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

/* HELPER FUNCTIONS */

DEBUG_COPY(Stm32f2xxRcc,1);                                                                                

/* Enable the peripheral clock if the specified bit is set in the value. */
static void stm32_rcc_periph_enable(
                                    Stm32f2xxRcc *s,
                                    uint32_t new_value,
                                    bool init,
                                    int periph,
                                    uint32_t bit_mask)
{
    //printf("rcc set 0x%x %s %d %x: %sable\n", new_value, s->PERIPHCLK[periph]->name, periph, bit_mask, IS_BIT_SET(new_value, bit_mask)?"en":"dis");
    clktree_set_enabled(&s->PERIPHCLK[periph], IS_BIT_SET(new_value, bit_mask));
}





/* REGISTER IMPLEMENTATION */

/* Read the configuration register.
   NOTE: Not implemented: CSS, PLLI2S, clock calibration and trimming. */
static uint32_t stm32_rcc_RCC_CR_read(Stm32f2xxRcc *s)
{
    /* Get the status of the clocks. */
    const bool PLLON = clktree_is_enabled(&s->PLLCLK);
    const bool HSEON = clktree_is_enabled(&s->HSECLK);
    const bool HSION = clktree_is_enabled(&s->HSICLK);
    const bool PLLI2SON = clktree_is_enabled(&s->PLLI2SCLK);

    /* build the register value based on the clock states.  If a clock is on,
     * then its ready bit is always set.
     */
    return (
        GET_BIT_MASK(RCC_CR_PLLRDY_BIT, PLLON) |
        GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLON) |

        GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
        GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |

        GET_BIT_MASK(RCC_CR_HSIRDY_BIT, HSION) |
        GET_BIT_MASK(RCC_CR_HSION_BIT, HSION) |

        GET_BIT_MASK(RCC_CR_PLLI2SRDY_BIT, PLLI2SON) |
        GET_BIT_MASK(RCC_CR_PLLI2SON_BIT, PLLI2SON)
    );
}

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    bool new_PLLON, new_HSEON, new_HSION, new_PLLI2SON;

    new_PLLON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
    if((clktree_is_enabled(&s->PLLCLK) && !new_PLLON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(&s->PLLCLK, new_PLLON);

    new_HSEON = IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
    if((clktree_is_enabled(&s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->HSECLK, new_HSEON);

    new_HSION = IS_BIT_SET(new_value, RCC_CR_HSION_BIT);
    if((clktree_is_enabled(&s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(&s->HSICLK, new_HSION);

    new_PLLI2SON = IS_BIT_SET(new_value, RCC_CR_PLLI2SON_BIT);
    clktree_set_enabled(&s->PLLI2SCLK, new_PLLI2SON);

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_CSSON_BIT, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSICAL_MASK, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSITRIM_MASK, RCC_CR_RESET_VALUE);
}


static uint32_t stm32_rcc_RCC_PLLCFGR_read(Stm32f2xxRcc *s)
{
    return s->RCC_PLLCFGR;
}

static void stm32_rcc_RCC_PLLCFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PLLM division factor */
    const uint8_t new_PLLM = (new_value & RCC_PLLCFGR_PLLM_MASK) >> RCC_PLLCFGR_PLLM_START;
    if (new_PLLM <= 1) {
        printf("PLLM division factor cannot be 0 or 1. Given: %u", new_PLLM);
    }

    /* PLLN multiplication factor */
    const uint16_t new_PLLN = (new_value & RCC_PLLCFGR_PLLN_MASK) >> RCC_PLLCFGR_PLLN_START;
    if (new_PLLN <= 1 || new_PLLN >= 433) {
        printf("PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", new_PLLN);
    }

    /* PLLSRC */
    const uint8_t new_PLLSRC = IS_BIT_SET(new_value, RCC_PLLCFGR_PLLSRC_BIT);

    /* PPLP division factor */
    const uint8_t new_PLLP = 2 + (2 * ((new_value & RCC_PLLCFGR_PLLP_MASK) >> RCC_PLLCFGR_PLLP_START));

    /* Warn in case of illegal writes: */
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(&s->PLLCLK) /* && TODO: !clktree_is_enabled(s->PLLI2SCLK) */);
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled";
            if (new_PLLM != s->RCC_PLLCFGR_PLLM) {
                printf(warning_fmt, "PLLM");
            }
            if (new_PLLN != s->RCC_PLLCFGR_PLLN) {
                printf(warning_fmt, "PLLN");
            }
            if (new_PLLSRC != s->RCC_PLLCFGR_PLLSRC) {
                printf(warning_fmt, "PLLSRC");
            }
        }
    }

    /* Save new register value */
    s->RCC_PLLCFGR = new_value;

    /* Set the new values: */
    s->RCC_PLLCFGR_PLLM = new_PLLM;
    s->RCC_PLLCFGR_PLLN = new_PLLN;
    clktree_set_scale(&s->PLLM, new_PLLN, new_PLLM);

    s->RCC_PLLCFGR_PLLSRC = new_PLLSRC;
    clktree_set_selected_input(&s->PLLM, new_PLLSRC);

    s->RCC_PLLCFGR_PLLP = new_PLLP;
    clktree_set_scale(&s->PLLCLK, 1, new_PLLP);

    WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLQ_MASK, RCC_PLLCFGR_RESET_VALUE);
}


static uint32_t stm32_rcc_RCC_PLLI2SCFGR_read(Stm32f2xxRcc *s)
{
    return s->RCC_PLLI2SCFGR;
}


static void stm32_rcc_RCC_PLLI2SCFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PLLR division factor */
    const uint16_t new_PLLR = (new_value & RCC_PLLI2SCFGR_PLLR_MASK) >> RCC_PLLI2SCFGR_PLLR_START;
    if (new_PLLR < 2 || new_PLLR > 7) {
        printf("PLLR multiplication factor must be between 2 and 7. Given: %u", new_PLLR);
    }

    /* PLLQ division factor */
    const uint16_t new_PLLQ = (new_value & RCC_PLLI2SCFGR_PLLQ_MASK) >> RCC_PLLI2SCFGR_PLLQ_START;
    if (new_PLLQ > 15) {
        printf("PLLQ multiplication factor must be between 0 and 15 "
                 "(inclusive). Given: %u", new_PLLQ);
    }

    /* PLLN multiplication factor */
    const uint16_t new_PLLN = (new_value & RCC_PLLI2SCFGR_PLLN_MASK) >> RCC_PLLI2SCFGR_PLLN_START;
    if (new_PLLN < 2 || new_PLLN > 433) {
        printf("PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", new_PLLN);
    }


    /* Warn in case of illegal writes: */
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(&s->PLLI2SCLK) /* && TODO: !clktree_is_enabled(s->PLLI2SCLK) */);
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled";
            if (new_PLLR != s->RCC_PLLI2SCFGR_PLLR) {
                printf(warning_fmt, "PLLR");
            }
            if (new_PLLQ != s->RCC_PLLI2SCFGR_PLLQ) {
                printf(warning_fmt, "PLLQ");
            }
            if (new_PLLN != s->RCC_PLLCFGR_PLLN) {
                printf(warning_fmt, "PLLN");
            }
        }
    }

    /* Save new register value */
    s->RCC_PLLI2SCFGR = new_value;

    /* Set the new values: */
    s->RCC_PLLI2SCFGR_PLLR = new_PLLR;
    s->RCC_PLLI2SCFGR_PLLQ = new_PLLQ;
    s->RCC_PLLI2SCFGR_PLLN = new_PLLN;
    clktree_set_scale(&s->PLLI2SM, new_PLLN, s->RCC_PLLCFGR_PLLM );

    clktree_set_scale(&s->PLLI2SCLK, 1, new_PLLR);
    WARN_UNIMPLEMENTED(new_value, RCC_PLLI2SCFGR_PLLQ_MASK, RCC_PLLI2SCFGR_RESET_VALUE);
}


static uint32_t stm32_rcc_RCC_CFGR_read(Stm32f2xxRcc *s)
{
    return
    (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
    (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
    (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}

static void stm32_rcc_RCC_CFGR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    /* PPRE2 */
    s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
    if(s->RCC_CFGR_PPRE2 < 0x4) {
        clktree_set_scale(&s->PCLK2, 1, 1);
    } else {
        clktree_set_scale(&s->PCLK2, 1, 2 * (s->RCC_CFGR_PPRE2 - 3));
    }

    /* PPRE1 */
    s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
        clktree_set_scale(&s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(&s->PCLK1, 1, 2 * (s->RCC_CFGR_PPRE1 - 3));
    }

    /* HPRE */
    s->RCC_CFGR_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
    if(s->RCC_CFGR_HPRE < 8) {
        clktree_set_scale(&s->HCLK, 1, 1);
    } else {
        clktree_set_scale(&s->HCLK, 1, 2 * (s->RCC_CFGR_HPRE - 7));
    }

    /* SW */
    s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
    switch(s->RCC_CFGR_SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(&s->SYSCLK, s->RCC_CFGR_SW);
            break;
        default:
            printf("Invalid input selected for SYSCLK");
            break;
    }

    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CFGR_I2CSRC_BIT, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_RTCPRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_SWS_MASK, RCC_CFGR_RESET_VALUE);
}

static uint32_t stm32_rcc_RCC_CIR_read(Stm32f2xxRcc *s)
{
    return s->RCC_CIR;
}

static void stm32_rcc_RCC_CIR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSC_BIT, RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYF_BIT , RCC_CIR_RESET_VALUE);
}

static uint32_t stm32_rcc_RCC_AHB1ENR_read(Stm32f2xxRcc *s)
{
    return s->RCC_AHB1ENR;
}

static uint32_t stm32_rcc_RCC_AHB2ENR_read(Stm32f2xxRcc *s)
{
    return s->RCC_AHB2ENR;
}

static uint32_t stm32_rcc_RCC_AHB3ENR_read(Stm32f2xxRcc *s)
{
    return s->RCC_AHB3ENR;
}

static void stm32_rcc_RCC_AHB1ENR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(&s->PERIPHCLK[STM32_DMA2], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA2EN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_DMA1], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA1EN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_CRC], IS_BIT_SET(new_value, RCC_AHB1ENR_CRCEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOK], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOKEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOJ], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOJEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOI], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOH], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOHEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOG], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOGEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOF], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOFEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOE], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOEEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOD], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIODEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOC], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOCEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOB], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOBEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_GPIOA], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOAEN_BIT));

    s->RCC_AHB1ENR = new_value;

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSULPIEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACPTPEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACRXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACTXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_BKPSRAMEN_BIT, RCC_AHB1ENR_RESET_VALUE);
}

static void stm32_rcc_RCC_AHB2ENR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(&s->PERIPHCLK[STM32_DCMI_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_DCMIEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_CRYP_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_CRYPEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_HASH_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_HASHEN_BIT));
    clktree_set_enabled(&s->PERIPHCLK[STM32_RNG_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_RNGEN_BIT));
    s->RCC_AHB2ENR = new_value;
}

static void stm32_rcc_RCC_AHB3ENR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(&s->PERIPHCLK[STM32_FSMC],
                        IS_BIT_SET(new_value, RCC_AHB3ENR_FSMCEN_BIT));
    s->RCC_AHB3ENR = new_value;
}

/* Write the APB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB2ENR_write(Stm32f2xxRcc *s, uint32_t new_value,
                                        bool init)
{
    /* TODO: enable/disable missing peripherals */
    stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG, RCC_APB2ENR_SYSCFGEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART1, RCC_APB2ENR_USART1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART6, RCC_APB2ENR_USART6EN_BIT);

    stm32_rcc_periph_enable(s, new_value, init, STM32_ADC1, RCC_APB2ENR_ADC1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_ADC2, RCC_APB2ENR_ADC2EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_ADC3, RCC_APB2ENR_ADC3EN_BIT);

    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM1, RCC_APB2ENR_TIM1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM8, RCC_APB2ENR_TIM8EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM9, RCC_APB2ENR_TIM9EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM10, RCC_APB2ENR_TIM10EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM11, RCC_APB2ENR_TIM11EN_BIT);

    s->RCC_APB2ENR = new_value & RCC_APB2ENR_MASK;
}

/* Write the APB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB1ENR_write(Stm32f2xxRcc *s, uint32_t new_value,
                                        bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART8,
                            RCC_APB1ENR_USART8EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART7,
                            RCC_APB1ENR_USART7EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART5,
                            RCC_APB1ENR_USART5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART4,
                            RCC_APB1ENR_USART4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART3,
                            RCC_APB1ENR_USART3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART2,
                            RCC_APB1ENR_USART2EN_BIT);

    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM2,
        RCC_APB1ENR_TIM2EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM3,
        RCC_APB1ENR_TIM3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM4,
        RCC_APB1ENR_TIM4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM5,
        RCC_APB1ENR_TIM5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM6,
        RCC_APB1ENR_TIM6EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM7,
        RCC_APB1ENR_TIM7EN_BIT);        
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM12,
        RCC_APB1ENR_TIM12EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM13,
        RCC_APB1ENR_TIM13EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM14,
        RCC_APB1ENR_TIM14EN_BIT);

    /* 0b00110110111111101100100111111111 */
    s->RCC_APB1ENR = new_value & 0x36fec9ff;
}

static uint32_t stm32_rcc_RCC_BDCR_read(Stm32f2xxRcc *s)
{
    bool lseon = clktree_is_enabled(&s->LSECLK);

    return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) |
    GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon) | 0x100; /* XXX force LSE */
}

static void stm32_rcc_RCC_BDCR_writeb0(Stm32f2xxRcc *s, uint8_t new_value, bool init)
{
    clktree_set_enabled(&s->LSECLK, IS_BIT_SET(new_value, RCC_BDCR_LSEON_BIT));
    
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_LSEBYP_BIT, RCC_BDCR_RESET_VALUE);
}

static void stm32_rcc_RCC_BDCR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    stm32_rcc_RCC_BDCR_writeb0(s, new_value & 0xff, init);

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_BDRST_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_RTCEN_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RESET_VALUE);
}

/* Works the same way as stm32_rcc_RCC_CR_read */
static uint32_t stm32_rcc_RCC_CSR_read(Stm32f2xxRcc *s)
{
    bool lseon = clktree_is_enabled(&s->LSICLK);

    return GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, lseon) |
    GET_BIT_MASK(RCC_CSR_LSION_BIT, lseon);
}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(Stm32f2xxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(&s->LSICLK, IS_BIT_SET(new_value, RCC_CSR_LSION_BIT));
}



static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    switch (offset) {
        case RCC_CR_OFFSET:
            return stm32_rcc_RCC_CR_read(s);
        case RCC_PLLCFGR_OFFSET:
            return stm32_rcc_RCC_PLLCFGR_read(s);
        case RCC_CFGR_OFFSET:
            return stm32_rcc_RCC_CFGR_read(s);
        case RCC_CIR_OFFSET:
            return stm32_rcc_RCC_CIR_read(s);
        case RCC_AHB1RSTR_OFFSET:
        case RCC_AHB2RSTR_OFFSET:
        case RCC_AHB3RSTR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_APB1RSTR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_APB2RSTR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_AHB1ENR_OFFSET:
            return stm32_rcc_RCC_AHB1ENR_read(s);
        case RCC_AHB2ENR_OFFSET:
            return stm32_rcc_RCC_AHB2ENR_read(s);
        case RCC_AHB3ENR_OFFSET:
            return stm32_rcc_RCC_AHB3ENR_read(s);
        case RCC_APB1ENR_OFFSET:
            return s->RCC_APB1ENR;
        case RCC_APB2ENR_OFFSET:
            return s->RCC_APB2ENR;
        case RCC_AHB1LPENR_OFFSET:
        case RCC_AHB2LPENR_OFFSET:
        case RCC_AHB3LPENR_OFFSET:
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_BDCR_OFFSET:
            return stm32_rcc_RCC_BDCR_read(s);
        case RCC_CSR_OFFSET:
            return stm32_rcc_RCC_CSR_read(s);
        case RCC_SSCGR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_PLLI2SCFGR_OFFSET:
            return stm32_rcc_RCC_PLLI2SCFGR_read(s);
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
    return 0;
}

static void stm32_rcc_writeb(void *opaque, hwaddr offset, uint64_t value)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    switch (offset) {
    case RCC_BDCR_OFFSET:
        stm32_rcc_RCC_BDCR_writeb0(s, value, false);
        break;
    default:
        STM32_BAD_REG(offset, 1U);
        break;
    }
}

static void stm32_rcc_RST_write(Stm32f2xxRcc *s, uint32_t mask, const uint8_t vectors[32]) {
    for (int i=0; i<32; i++) {
        if (mask & 1U<<i && vectors[i] != 0 ) {
            qemu_irq_pulse(s->reset[vectors[i]]);
        }
    }
}

static void stm32_rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value)
{
    Stm32f2xxRcc *s = (Stm32f2xxRcc *)opaque;

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_PLLCFGR_OFFSET:
            stm32_rcc_RCC_PLLCFGR_write(s, value, false);
            break;
        case RCC_CFGR_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CIR_OFFSET:
            stm32_rcc_RCC_CIR_write(s, value, false);
            break;
        case RCC_APB1RSTR_OFFSET:
            stm32_rcc_RST_write(s, value, APB1RST_PERIPHS);
            break;
        case RCC_APB2RSTR_OFFSET:
            stm32_rcc_RST_write(s, value, APB2RST_PERIPHS);
            break;
        case RCC_AHB1RSTR_OFFSET:
            stm32_rcc_RST_write(s, value, AHB1RST_PERIPHS);
            break;
        case RCC_AHB2RSTR_OFFSET:
            stm32_rcc_RST_write(s, value, AHB2RST_PERIPHS);
            break;
        case RCC_AHB3RSTR_OFFSET:
            stm32_rcc_RST_write(s, value, AHB3RST_PERIPHS);
            break;
        case RCC_AHB1ENR_OFFSET:
            stm32_rcc_RCC_AHB1ENR_write(s, value, false);
            break;
        case RCC_AHB2ENR_OFFSET:
            stm32_rcc_RCC_AHB2ENR_write(s, value, false);
            break;
        case RCC_AHB3ENR_OFFSET:
            stm32_rcc_RCC_AHB3ENR_write(s, value, false);
            break;
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_APB2ENR_OFFSET:
            stm32_rcc_RCC_APB2ENR_write(s, value, false);
            break;
        case RCC_APB1ENR_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_AHB1LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB1LPENR_OFFSET\n");
            break;
        case RCC_AHB2LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB2LPENR_OFFSET\n");
            break;
        case RCC_AHB3LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB3LPENR_OFFSET\n");
            break;
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        case RCC_SSCGR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_PLLI2SCFGR_OFFSET:
            stm32_rcc_RCC_PLLI2SCFGR_write(s, value, false);
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
            stm32_rcc_writew(opaque, offset, value);
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
    Stm32f2xxRcc *s = STM32F2XX_RCC(dev);

    stm32_rcc_RCC_CR_write(s, RCC_CR_RESET_VALUE, true);
    stm32_rcc_RCC_PLLCFGR_write(s, RCC_PLLCFGR_RESET_VALUE, true);
    stm32_rcc_RCC_CFGR_write(s, RCC_CFGR_RESET_VALUE, true);
    stm32_rcc_RCC_APB2ENR_write(s, RCC_APB2ENR_RESET_VALUE, true);
    stm32_rcc_RCC_APB1ENR_write(s, RCC_APB1ENR_RESET_VALUE, true);
    stm32_rcc_RCC_BDCR_write(s, RCC_BDCR_RESET_VALUE, true);
    stm32_rcc_RCC_CSR_write(s, RCC_CSR_RESET_VALUE, true);
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

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
#endif
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_realize(DeviceState *dev, Error **errp)
{
    Stm32f2xxRcc *s = STM32F2XX_RCC(dev);
    int i;
    s->hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i].is_initialized = false;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    clktree_create_src_clk(&s->HSICLK, "HSI", HSI_FREQ, false);
    clktree_create_src_clk(&s->LSICLK, "LSI", LSI_FREQ, false);
    clktree_create_src_clk(&s->HSECLK, "HSE", s->osc_freq, false);
    clktree_create_src_clk(&s->LSECLK, "LSE", s->osc32_freq, false);

    // Moved into PERIPHCLK so it can be queried
    // s->IWDGCLK = clktree_create_clk("IWDGCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0,
    //                                 s->LSICLK, NULL);
    clktree_create_clk(&s->RTCCLK, "RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ,
                                   CLKTREE_NO_INPUT, &s->LSECLK, &s->LSICLK, &s->HSECLK, NULL);

    clktree_create_clk(&s->PLLM, "PLLM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, &s->HSICLK,
                                 &s->HSECLK, NULL);
    clktree_create_clk(&s->PLLCLK, "PLLCLK", 1, 2, false, 120000000, 0, &s->PLLM, NULL);
    clktree_create_clk(&s->PLL48CLK, "PLL48CLK", 1, 1, false, 48000000, 0, &s->PLLM, NULL);

    clktree_create_clk(&s->PLLI2SM, "PLLI2SM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, &s->PLLM, NULL);
    clktree_create_clk(&s->PLLI2SCLK, "PLLI2SCLK", 1, 2, false, 120000000, 0, &s->PLLI2SM, NULL);

    clktree_create_clk(&s->SYSCLK, "SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   &s->HSICLK, &s->HSECLK, &s->PLLCLK, NULL);

    // HCLK: to AHB bus, core memory and DMA
    clktree_create_clk(&s->HCLK, "HCLK", 0, 1, true, 168000000, 0, &s->SYSCLK, NULL);
    clktree_adduser(&s->HCLK, s->hclk_upd_irq[0]);

    // Clock source for APB1 peripherals:

    clktree_create_clk(&s->PCLK1, "PCLK1", 0, 1, true, 30000000, 0, &s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    clktree_create_clk(&s->PCLK2, "PCLK2", 0, 1, true, 60000000, 0, &s->HCLK, NULL);

    /* Peripheral clocks */
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOA], "GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOB], "GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOC], "GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOD], "GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOE], "GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOF], "GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOG], "GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOH], "GPIOH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOI], "GPIOI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOJ], "GPIOJ", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_GPIOK], "GPIOK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PERIPHCLK[STM32_CRC], "CRC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PERIPHCLK[STM32_DMA1], "DMA1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_DMA2], "DMA2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    
    clktree_create_clk(&s->PERIPHCLK[STM32_SYSCFG], "SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);

    clktree_create_clk(&s->PERIPHCLK[STM32_UART1], "UART1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART2], "UART2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART3], "UART3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART4], "UART4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART5], "UART5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART6], "UART6", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART7], "UART7", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_UART8], "UART8", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    
    
    clktree_create_clk(&s->PERIPHCLK[STM32_ADC1], "ADC1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_ADC2], "ADC2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_ADC3], "ADC3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);

    // Timers run at 2x the APB speed 
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM1], "TIM1", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM2], "TIM2", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM3], "TIM3", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM4], "TIM4", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM5], "TIM5", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM6], "TIM6", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM7], "TIM7", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM8], "TIM8", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM9], "TIM9", 2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM10], "TIM10",2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM11], "TIM11",2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK2, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM12], "TIM12",2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM13], "TIM13",2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_TIM14], "TIM14",2, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->PCLK1, NULL);


    clktree_create_clk(&s->PERIPHCLK[STM32_IWDG], "IWDGCLK", 1, 1, true, CLKTREE_NO_MAX_FREQ, 0,
                                    &s->LSICLK, NULL);

    clktree_create_clk(&s->PERIPHCLK[STM32_DCMI_PERIPH], "DCMI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_CRYP_PERIPH], "CRYP", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_HASH_PERIPH], "HASH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    clktree_create_clk(&s->PERIPHCLK[STM32_RNG_PERIPH], "RNG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);

    clktree_create_clk(&s->PERIPHCLK[STM32_FSMC], "FSMC", 1, 2, false, CLKTREE_NO_MAX_FREQ, 0, &s->HCLK, NULL);
    // Reset IRQs:
    qdev_init_gpio_out_named(DEVICE(dev),s->reset,"reset",STM32_PERIPH_COUNT);
}



static void stm32_rcc_init(Object *obj)
{

    Stm32f2xxRcc *s = STM32F2XX_RCC(obj);

    memory_region_init_io(&s->iomem, obj, &stm32_rcc_ops, s,
                          "rcc", 0x40023BFF - 0x40023800 + 1);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    // stm32_rcc_init_clk(s);
}


static Property stm32_rcc_properties[] = {
    DEFINE_PROP_UINT32("osc_freq", Stm32f2xxRcc, osc_freq, 0),
    DEFINE_PROP_UINT32("osc32_freq", Stm32f2xxRcc, osc32_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};

static int rcc_pre_save(void *opaque) {
    DEBUG_TAKE(opaque,0);
    return 0;
}

#define DEBUG_CHK_CLK(name) \
    DEBUG_CHECK(name.enabled); \
    DEBUG_CHECK(name.input_freq); \
    DEBUG_CHECK(name.output_freq); \
    DEBUG_CHECK(name.max_output_freq); \
    DEBUG_CHECK(name.multiplier); \
    DEBUG_CHECK(name.divisor); \
    DEBUG_CHECK(name.user_count); \
    DEBUG_CHECK(name.output_count); \
    DEBUG_CHECK(name.input_count); \
    DEBUG_CHECK(name.selected_input); 

static int rcc_post_load(void *opaque, int version) {
    DEBUG_CAST_ONLY(Stm32f2xxRcc *s = STM32F2XX_RCC(opaque));
    DEBUG_INDEX(0);
    for (int i=0; i<STM32_PERIPH_COUNT; i++) {
        DEBUG_CHK_CLK(PERIPHCLK[i]);
    }
    DEBUG_CHK_CLK(HSICLK);
    DEBUG_CHK_CLK(HSECLK);
    DEBUG_CHK_CLK(LSECLK);
    DEBUG_CHK_CLK(LSICLK);
    DEBUG_CHK_CLK(SYSCLK);
    DEBUG_CHK_CLK(IWDGCLK);
    DEBUG_CHK_CLK(RTCCLK);
    DEBUG_CHK_CLK(PLLM);
    DEBUG_CHK_CLK(PLLCLK);
    DEBUG_CHK_CLK(PLL48CLK);
    DEBUG_CHK_CLK(PLLI2SM);
    DEBUG_CHK_CLK(PLLI2SCLK);
    DEBUG_CHK_CLK(HCLK);
    DEBUG_CHK_CLK(PCLK1);
    DEBUG_CHK_CLK(PCLK2);
    DEBUG_CHECK(osc_freq);
    DEBUG_CHECK(osc32_freq);
    DEBUG_CHECK(RCC_CIR);
    DEBUG_CHECK(RCC_APB1ENR);
    DEBUG_CHECK(RCC_APB2ENR);
    DEBUG_CHECK(RCC_CFGR_PPRE1);
    DEBUG_CHECK(RCC_CFGR_PPRE2);
    DEBUG_CHECK(RCC_CFGR_HPRE);
    DEBUG_CHECK(RCC_AHB1ENR);
    DEBUG_CHECK(RCC_AHB2ENR);
    DEBUG_CHECK(RCC_AHB3ENR);
    DEBUG_CHECK(RCC_CFGR_SW);
    DEBUG_CHECK(RCC_PLLCFGR);
    DEBUG_CHECK(RCC_PLLI2SCFGR);
    DEBUG_CHECK(RCC_PLLCFGR_PLLM);
    DEBUG_CHECK(RCC_PLLCFGR_PLLP);
    DEBUG_CHECK(RCC_PLLCFGR_PLLSRC);
    DEBUG_CHECK(RCC_PLLCFGR_PLLN);
    DEBUG_CHECK(RCC_PLLI2SCFGR_PLLR);
    DEBUG_CHECK(RCC_PLLI2SCFGR_PLLQ);
    DEBUG_CHECK(RCC_PLLI2SCFGR_PLLN);
    return 0;
}

static const VMStateDescription vmstate_stm32f2xx_rcc_clk = {
    .name = TYPE_STM32F2XX_RCC "-clk",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enabled, struct Clk),
        VMSTATE_UINT32(input_freq,struct Clk),
        VMSTATE_UINT32(output_freq,struct Clk),
        VMSTATE_UINT32(max_output_freq,struct Clk),
        VMSTATE_UINT16(multiplier,struct Clk),
        VMSTATE_UINT16(divisor,struct Clk),
        VMSTATE_UINT8(user_count,struct Clk),
        VMSTATE_UINT8(output_count,struct Clk),
        VMSTATE_UINT8(input_count,struct Clk),
        VMSTATE_INT16(selected_input,struct Clk),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_stm32f2xx_rcc = {
    .name = TYPE_STM32F2XX_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = rcc_pre_save,
    .post_load = rcc_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(osc_freq, Stm32f2xxRcc),
        VMSTATE_UINT32(osc32_freq, Stm32f2xxRcc),
        VMSTATE_STRUCT_ARRAY(PERIPHCLK, Stm32f2xxRcc, STM32_PERIPH_COUNT, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HSICLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HSECLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(LSECLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(LSICLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(SYSCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(IWDGCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(RTCCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLM,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLL48CLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLI2SM,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PLLI2SCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HCLK,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK1,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_STRUCT(PCLK2,Stm32f2xxRcc, 1, vmstate_stm32f2xx_rcc_clk, Clk_t),
        VMSTATE_UINT32(RCC_CIR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_APB1ENR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_APB2ENR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_CFGR_PPRE1,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_CFGR_PPRE2,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_CFGR_HPRE,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_AHB1ENR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_AHB2ENR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_AHB3ENR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_CFGR_SW,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_PLLCFGR,Stm32f2xxRcc),
        VMSTATE_UINT32(RCC_PLLI2SCFGR,Stm32f2xxRcc),
        VMSTATE_UINT8(RCC_PLLCFGR_PLLM,Stm32f2xxRcc),
        VMSTATE_UINT8(RCC_PLLCFGR_PLLP,Stm32f2xxRcc),
        VMSTATE_UINT8(RCC_PLLCFGR_PLLSRC,Stm32f2xxRcc),
        VMSTATE_UINT16(RCC_PLLCFGR_PLLN,Stm32f2xxRcc),
        VMSTATE_UINT8(RCC_PLLI2SCFGR_PLLR,Stm32f2xxRcc),
        VMSTATE_UINT8(RCC_PLLI2SCFGR_PLLQ,Stm32f2xxRcc),
        VMSTATE_UINT16(RCC_PLLI2SCFGR_PLLN,Stm32f2xxRcc),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_rcc_reset;
    dc->realize = stm32_rcc_realize;
    dc->vmsd = &vmstate_stm32f2xx_rcc;
    device_class_set_props(dc, stm32_rcc_properties);
}

static TypeInfo stm32_rcc_info = {
    .name  = TYPE_STM32F2XX_RCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32f2xxRcc),
    .class_init = stm32_rcc_class_init,
    .instance_init = stm32_rcc_init
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
