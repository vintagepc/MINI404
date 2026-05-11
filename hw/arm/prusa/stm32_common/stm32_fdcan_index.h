/*-
 * CAN module for STM32 SOCs in QEMU.
 *
 * Currently known to be common to the following chips:
 * - STM32C092
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
enum stm32_fdcan_ri
{
	RI_CREL                 = (0x000/4U), /* FDCAN Core Release register, */
	RI_ENDN                 = (0x004/4U), /* FDCAN Endian register, */
	RI_DBTP                 = (0x00C/4U), /* FDCAN Data Bit Timing & Prescaler register, */
	RI_TEST                 = (0x010/4U), /* FDCAN Test register, */
	RI_RWD                  = (0x014/4U), /* FDCAN RAM Watchdog register, */
	RI_CCCR                 = (0x018/4U), /* FDCAN CC Control register, */
	RI_NBTP                 = (0x01C/4U), /* FDCAN Nominal Bit Timing & Prescaler register, */
	RI_TSCC                 = (0x020/4U), /* FDCAN Timestamp Counter Configuration register, */
	RI_TSCV                 = (0x024/4U), /* FDCAN Timestamp Counter Value register, */
	RI_TOCC                 = (0x028/4U), /* FDCAN Timeout Counter Configuration register, */
	RI_TOCV                 = (0x02C/4U), /* FDCAN Timeout Counter Value register, */
	RI_ECR                  = (0x040/4U), /* FDCAN Error Counter register, */
	RI_PSR                  = (0x044/4U), /* FDCAN Protocol Status register, */
	RI_TDCR                 = (0x048/4U), /* FDCAN Transmitter Delay Compensation register, */
	RI_IR                   = (0x050/4U), /* FDCAN Interrupt register, */
	RI_IE                   = (0x054/4U), /* FDCAN Interrupt Enable register, */
	RI_ILS                  = (0x058/4U), /* FDCAN Interrupt Line Select register, */
	RI_ILE                  = (0x05C/4U), /* FDCAN Interrupt Line Enable register, */
	RI_RXGFC                = (0x080/4U), /* FDCAN Global Filter Configuration register, */
	RI_XIDAM                = (0x084/4U), /* FDCAN Extended ID AND Mask register, */
	RI_HPMS                 = (0x088/4U), /* FDCAN High Priority Message Status register, */
	RI_RXF0S                = (0x090/4U), /* FDCAN Rx FIFO 0 Status register, */
	RI_RXF0A                = (0x094/4U), /* FDCAN Rx FIFO 0 Acknowledge register, */
	RI_RXF1S                = (0x098/4U), /* FDCAN Rx FIFO 1 Status register, */
	RI_RXF1A                = (0x09C/4U), /* FDCAN Rx FIFO 1 Acknowledge register, */
	RI_TXBC                 = (0x0C0/4U), /* FDCAN Tx Buffer Configuration register, */
	RI_TXFQS                = (0x0C4/4U), /* FDCAN Tx FIFO/Queue Status register, */
	RI_TXBRP                = (0x0C8/4U), /* FDCAN Tx Buffer Request Pending register, */
	RI_TXBAR                = (0x0CC/4U), /* FDCAN Tx Buffer Add Request register, */
	RI_TXBCR                = (0x0D0/4U), /* FDCAN Tx Buffer Cancellation Request register, */
	RI_TXBTO                = (0x0D4/4U), /* FDCAN Tx Buffer Transmission Occurred register, */
	RI_TXBCF                = (0x0D8/4U), /* FDCAN Tx Buffer Cancellation Finished register, */
	RI_TXBTIE               = (0x0DC/4U), /* FDCAN Tx Buffer Transmission Interrupt Enable register, */
	RI_TXBCIE               = (0x0E0/4U), /* FDCAN Tx Buffer Cancellation Finished Interrupt Enable register, */
	RI_TXEFS                = (0x0E4/4U), /* FDCAN Tx Event FIFO Status register, */
	RI_TXEFA                = (0x0E8/4U), /* FDCAN Tx Event FIFO Acknowledge register, */
	RI_CKDIV                = (0x100/4U), /* FDCAN clock divider register, */
	RI_END
};

