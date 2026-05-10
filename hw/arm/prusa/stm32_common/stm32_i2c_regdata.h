/*-
 * I2C module for STM32 SOCs in QEMU.
 *
 * Currently known to be common to the following chips:
 * - STM32C092
 * - STM32F030
 * - STM32G070
 * - STM32H503
 *
 * Copyright (c) 2026 VintagePC <github.com/vinagepc>
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once
enum stm32_i2c_ri
{
	RI_CR1                  = (0x00 /4U), /* I2C Control register 1, */
	RI_CR2                  = (0x04 /4U), /* I2C Control register 2, */
	RI_OAR1                 = (0x08 /4U), /* I2C Own address 1 register, */
	RI_OAR2                 = (0x0C /4U), /* I2C Own address 2 register, */
	RI_TIMINGR              = (0x10 /4U), /* I2C Timing register, */
	RI_TIMEOUTR             = (0x14 /4U), /* I2C Timeout register, */
	RI_ISR                  = (0x18 /4U), /* I2C Interrupt and status register, */
	RI_ICR                  = (0x1C /4U), /* I2C Interrupt clear register, */
	RI_PECR                 = (0x20 /4U), /* I2C PEC register, */
	RI_RXDR                 = (0x24 /4U), /* I2C Receive data register, */
	RI_TXDR                 = (0x28 /4U), /* I2C Transmit data register, */
	RI_END
};

