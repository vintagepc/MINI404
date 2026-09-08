/*
 * MINI404 - Shared STM32 I2C bit-banging helpers for qtest test cases.
 *
 * This file is part of the MINI404 project, an open-source 3D printer simulator.
 * Copyright 2026 VintagePC <https://github.com/vintagepc>
 *
 * MINI404 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * MINI404 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MINI404. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32_I2C_TEST_HELPER_H
#define STM32_I2C_TEST_HELPER_H

#include "qemu/osdep.h"
#include "libqtest-single.h"

/* --------------------------------------------------------------------------
 * "Common" STM32 I2C peripheral: ISR/CR2/TXDR/RXDR style (G0/H5/C0 family,
 * stm32_common/stm32_i2c.c).
 * -------------------------------------------------------------------------- */

#define I2C_COM_CR1_OFF     0x00
#define I2C_COM_CR2_OFF     0x04
#define I2C_COM_ISR_OFF     0x18
#define I2C_COM_RXDR_OFF    0x24
#define I2C_COM_TXDR_OFF    0x28

#define I2C_COM_CR2_SADD(a)     ((a) & 0x3FFU)
#define I2C_COM_CR2_RD_WRN      (1U << 10)
#define I2C_COM_CR2_START       (1U << 13)
#define I2C_COM_CR2_NBYTES(n)   (((n) & 0xFFU) << 16)
#define I2C_COM_CR2_AUTOEND     (1U << 25)

#define I2C_COM_CR1_PE      (1U << 0)

#define I2C_COM_ISR_NACKF   (1U << 4)

/* Enable peripheral. */
static inline void i2c_common_enable(QTestState *ts, uint64_t base)
{
    qtest_writel(ts, base + I2C_COM_CR1_OFF, I2C_COM_CR1_PE);
}

/* Write reg (1B) + value (2B, MSB first) as one txn: START, addr(W), bytes, STOP. */
static inline void i2c_common_write_reg16(QTestState *ts, uint64_t base,
                                           uint8_t dev_addr, uint8_t reg,
                                           uint16_t value)
{
    uint32_t cr2 = I2C_COM_CR2_SADD(dev_addr) | I2C_COM_CR2_NBYTES(3) |
                   I2C_COM_CR2_AUTOEND | I2C_COM_CR2_START;
    qtest_writel(ts, base + I2C_COM_CR2_OFF, cr2);
    qtest_writel(ts, base + I2C_COM_TXDR_OFF, reg);
    qtest_writel(ts, base + I2C_COM_TXDR_OFF, (value >> 8) & 0xFF);
    qtest_writel(ts, base + I2C_COM_TXDR_OFF, value & 0xFF);
}

/*
 * Read reg (1B) via write-pointer then repeated-start read, 2 bytes back.
 * Extra RD_WRN priming write works around the model latching direction
 * from the prior CR2 state, not the one being written.
 */
static inline uint16_t i2c_common_read_reg16(QTestState *ts, uint64_t base,
                                              uint8_t dev_addr, uint8_t reg)
{
    uint32_t cr2_wr = I2C_COM_CR2_SADD(dev_addr) | I2C_COM_CR2_NBYTES(1) |
                      I2C_COM_CR2_START;
    qtest_writel(ts, base + I2C_COM_CR2_OFF, cr2_wr);
    qtest_writel(ts, base + I2C_COM_TXDR_OFF, reg);

    qtest_writel(ts, base + I2C_COM_CR2_OFF,
                 I2C_COM_CR2_SADD(dev_addr) | I2C_COM_CR2_RD_WRN |
                 I2C_COM_CR2_NBYTES(2) | I2C_COM_CR2_AUTOEND);
    qtest_writel(ts, base + I2C_COM_CR2_OFF,
                 I2C_COM_CR2_SADD(dev_addr) | I2C_COM_CR2_RD_WRN |
                 I2C_COM_CR2_NBYTES(2) | I2C_COM_CR2_AUTOEND | I2C_COM_CR2_START);

    uint32_t msb = qtest_readl(ts, base + I2C_COM_RXDR_OFF) & 0xFF;
    uint32_t lsb = qtest_readl(ts, base + I2C_COM_RXDR_OFF) & 0xFF;
    return (uint16_t)((msb << 8) | lsb);
}

/* --------------------------------------------------------------------------
 * Legacy STM32F2xx I2C peripheral: CR1/CR2/DR/SR1/SR2 style (F4 family,
 * stm32f407/stm32f2xx_i2c.c). Real hardware register set, unlike the ISR
 * style above -- SB/ADDR/BTF handshaking, one DR for both TX and RX.
 * Predates the reggen migration, so no generated offset header exists;
 * offsets below mirror the driver's private R_* defines.
 * -------------------------------------------------------------------------- */

#define I2C_F2XX_CR1_OFF   0x00
#define I2C_F2XX_DR_OFF    0x10

#define I2C_F2XX_CR1_PE     (1U << 0)
#define I2C_F2XX_CR1_START  (1U << 8)
#define I2C_F2XX_CR1_STOP   (1U << 9)

/* Enable peripheral. */
static inline void i2c_f2xx_enable(QTestState *ts, uint64_t base)
{
    qtest_writel(ts, base + I2C_F2XX_CR1_OFF, I2C_F2XX_CR1_PE);
}

/* Write len bytes to a 7-bit address: START, addr(W), data..., STOP. */
static inline void i2c_f2xx_write(QTestState *ts, uint64_t base, uint8_t addr,
                                   const uint8_t *data, size_t len)
{
    qtest_writel(ts, base + I2C_F2XX_CR1_OFF, I2C_F2XX_CR1_PE | I2C_F2XX_CR1_START);
    qtest_writel(ts, base + I2C_F2XX_DR_OFF, (uint32_t)(addr << 1));
    for (size_t i = 0; i < len; i++) {
        qtest_writel(ts, base + I2C_F2XX_DR_OFF, data[i]);
    }
    qtest_writel(ts, base + I2C_F2XX_CR1_OFF, I2C_F2XX_CR1_PE | I2C_F2XX_CR1_STOP);
}

/*
 * Read len bytes from a 7-bit address: START, addr(R), then the bytes.
 * STOP is raised right after the address ACK, before the last byte is
 * pulled from DR -- the real-hardware trick to NACK the final byte.
 * Side effect: the model always prefetches one extra byte from the slave
 * past the last one returned here, so a slave's internal read pointer
 * ends up len+1 further, not len -- harmless as long as callers re-send
 * the full address every transaction instead of relying on it.
 */
static inline void i2c_f2xx_read(QTestState *ts, uint64_t base, uint8_t addr,
                                  uint8_t *out, size_t len)
{
    qtest_writel(ts, base + I2C_F2XX_CR1_OFF, I2C_F2XX_CR1_PE | I2C_F2XX_CR1_START);
    qtest_writel(ts, base + I2C_F2XX_DR_OFF, (uint32_t)((addr << 1) | 1));
    for (size_t i = 0; i + 1 < len; i++) {
        out[i] = qtest_readl(ts, base + I2C_F2XX_DR_OFF) & 0xFF;
    }
    qtest_writel(ts, base + I2C_F2XX_CR1_OFF, I2C_F2XX_CR1_PE | I2C_F2XX_CR1_STOP);
    out[len - 1] = qtest_readl(ts, base + I2C_F2XX_DR_OFF) & 0xFF;
}

#endif
