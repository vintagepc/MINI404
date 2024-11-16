/*-
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
#ifndef STM32_COMMON_ADC_DATA_H
#define STM32_COMMON_ADC_DATA_H

// This is copy-pasted from the generated H503 info
// but with some changes to make it more generic.
enum reg_index {
	RI_ISR                  = (0x00 /4U), /* ADC interrupt and status register, */
	RI_IER                  = (0x04 /4U), /* ADC interrupt enable register, */
	RI_CR                   = (0x08 /4U), /* ADC control register, */
	RI_CFGR1                = (0x0C /4U), /* ADC configuration register 1, */
	RI_CFGR2                = (0x10 /4U), /* ADC configuration register 2, */
	RI_SMPR                 = (0x14 /4U), /* ADC sampling time register 1, */
	RI_SMPR2                = (0x18 /4U), /* ADC sampling time register 2, */
	RI_TR1                  = (0x20 /4U), /* ADC analog watchdog 1 threshold register, */
	RI_TR2                  = (0x24 /4U), /* ADC analog watchdog 2 threshold register, */
	RI_CHSELR               = (0x28 /4U), /* ADC group regular sequencer register, (F030, G070 only)*/
	RI_H503_TR3             = (0x28 /4U), /* ADC analog watchdog 3 threshold register, */
	RI_G070_TR3             = (0x2C /4U), /* ADC analog watchdog 3 threshold register, */
	RI_SQR1                 = (0x30 /4U), /* ADC group regular sequencer register 1, */
	RI_SQR2                 = (0x34 /4U), /* ADC group regular sequencer register 2, */
	RI_SQR3                 = (0x38 /4U), /* ADC group regular sequencer register 3, */
	RI_SQR4                 = (0x3C /4U), /* ADC group regular sequencer register 4, */
	RI_DR                   = (0x40 /4U), /* ADC group regular data register, */
	RI_JSQR                 = (0x4C /4U), /* ADC group injected sequencer register, */
	RI_OFR1                 = (0x60 /4U), /* ADC offset register 1, */
	RI_OFR2                 = (0x64 /4U), /* ADC offset register 2, */
	RI_OFR3                 = (0x68 /4U), /* ADC offset register 3, */
	RI_OFR4                 = (0x6C /4U), /* ADC offset register 4, */
	RI_JDR1                 = (0x80 /4U), /* ADC group injected rank 1 data register, */
	RI_JDR2                 = (0x84 /4U), /* ADC group injected rank 2 data register, */
	RI_JDR3                 = (0x88 /4U), /* ADC group injected rank 3 data register, */
	RI_JDR4                 = (0x8C /4U), /* ADC group injected rank 4 data register, */
	RI_AWD2CR               = (0xA0 /4U), /* ADC analog watchdog 2 configuration register, */
	RI_AWD3CR               = (0xA4 /4U), /* ADC analog watchdog 3 Configuration Register, */
	RI_DIFSEL               = (0xB0 /4U), /* ADC differential mode selection register, */
	RI_CALFACT              = (0xB4 /4U), /* ADC calibration factors, */
	RI_OR                   = (0xC8 /4U), /* ADC option register, */
	RI_END
};
#endif
