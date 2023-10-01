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

enum reg_index {
	RI_ISR,
	RI_IER,
	RI_CR,
	RI_CFGR1,
	RI_CFGR2,
	RI_SMPR,
	RI_AWD1TR = 8,
	RI_AWD2TR,
	RI_CHSELR,
	RI_AWD3TR,
	RI_DR = 16,
	RI_END
};
#endif
