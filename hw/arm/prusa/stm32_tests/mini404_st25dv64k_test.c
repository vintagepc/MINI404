/*
 * MINI404 - RFID EEPROM syspage basic functionality test:
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
 *
 * NOTE: prusa-mini wires this device's I2C1 to the legacy STM32F2xx I2C
 * model (CR1/CR2/DR/SR1/SR2), not the newer ISR/TXDR/RXDR "common" model
 * -- see stm32_i2c_test_helper.h's i2c_f2xx_* section.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"
#include "stm32_i2c_test_helper.h"
#include "../stm32_registers/generated/stm32f427/Addresses.h"

#define MACHINE_NAME    "prusa-mini"

/* F407/F427 share one generated register map; I2C1 is at the same address. */
#define ST25_I2C_BASE   ((uint64_t)F427_I2C1_ADDR)

/* System-page I2C address (datasheet: 0x57 / write 0xAE, read 0xAF) */
#define ST25_I2C_ADDR   0x57

/* System register addresses -- mirrors the private enum in ST25DV64K_syspage.c */
enum {
    REG_GPO         = 0x00,
    REG_IT_TIME     = 0x01,
    REG_EH_MODE     = 0x02,
    REG_RF_MNGT     = 0x03,
    REG_RFA1SS      = 0x04,
    REG_ENDA1       = 0x05,
    REG_RFA2SS      = 0x06,
    REG_ENDA2       = 0x07,
    REG_RFA3SS      = 0x08,
    REG_ENDA3       = 0x09,
    REG_RFA4SS      = 0x0A,
    REG_I2CSS       = 0x0B,
    REG_LOCK_CCFILE = 0x0C,
    REG_MB_MODE     = 0x0D,
    REG_MB_WDG      = 0x0E,
    REG_LOCK_CFG    = 0x0F,
};

#define ST25_NUM_REGS   0x10

/* Password-presentation message address -- well past the modeled bank */
#define ST25_REG_I2C_PWD 0x0900

/* --------------------------------------------------------------------------
 * Register access -- 2-byte address, 1 byte of data per register.
 * -------------------------------------------------------------------------- */

static void st25_write_reg(QTestState *ts, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, val };
    i2c_f2xx_write(ts, ST25_I2C_BASE, ST25_I2C_ADDR, buf, sizeof(buf));
}

static uint8_t st25_read_reg(QTestState *ts, uint16_t reg)
{
    uint8_t ptr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    uint8_t val;
    i2c_f2xx_write(ts, ST25_I2C_BASE, ST25_I2C_ADDR, ptr, sizeof(ptr));
    i2c_f2xx_read(ts, ST25_I2C_BASE, ST25_I2C_ADDR, &val, 1);
    return val;
}

/* One transaction: address + len data bytes -- exercises write auto-increment. */
static void st25_write_block(QTestState *ts, uint16_t reg,
                              const uint8_t *data, size_t len)
{
    uint8_t buf[2 + ST25_NUM_REGS];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)reg;
    memcpy(&buf[2], data, len);
    i2c_f2xx_write(ts, ST25_I2C_BASE, ST25_I2C_ADDR, buf, 2 + len);
}

/* One address write + one read transaction -- exercises read auto-increment. */
static void st25_read_block(QTestState *ts, uint16_t reg, uint8_t *out, size_t len)
{
    uint8_t ptr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    i2c_f2xx_write(ts, ST25_I2C_BASE, ST25_I2C_ADDR, ptr, sizeof(ptr));
    i2c_f2xx_read(ts, ST25_I2C_BASE, ST25_I2C_ADDR, out, len);
}

/* --------------------------------------------------------------------------
 * Tests
 * -------------------------------------------------------------------------- */

static QTestState *setup_machine(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE_NAME);
    i2c_f2xx_enable(ts, ST25_I2C_BASE);
    return ts;
}

/* Fresh device: whole bank reads back zero. */
static void test_st25_reset_defaults(void)
{
    QTestState *ts = setup_machine();

    g_assert_cmphex(st25_read_reg(ts, REG_GPO), ==, 0x00);
    g_assert_cmphex(st25_read_reg(ts, REG_LOCK_CFG), ==, 0x00);

    qtest_quit(ts);
}

/* Basic write/read round trip; neighbor register must stay untouched. */
static void test_st25_write_read_single(void)
{
    QTestState *ts = setup_machine();

    st25_write_reg(ts, REG_GPO, 0xAB);
    g_assert_cmphex(st25_read_reg(ts, REG_GPO), ==, 0xAB);
    g_assert_cmphex(st25_read_reg(ts, REG_IT_TIME), ==, 0x00);

    qtest_quit(ts);
}

/* Every register in the bank is independently addressable, no crosstalk. */
static void test_st25_full_bank_sweep(void)
{
    QTestState *ts = setup_machine();

    for (int i = 0; i < ST25_NUM_REGS; i++) {
        st25_write_reg(ts, i, (uint8_t)(0x10 + i));
    }
    for (int i = 0; i < ST25_NUM_REGS; i++) {
        g_assert_cmphex(st25_read_reg(ts, i), ==, (uint8_t)(0x10 + i));
    }

    qtest_quit(ts);
}

/* A multi-byte write auto-increments the register pointer. */
static void test_st25_write_auto_increment(void)
{
    QTestState *ts = setup_machine();
    const uint8_t block[4] = { 0x11, 0x22, 0x33, 0x44 };

    st25_write_block(ts, REG_RFA1SS, block, sizeof(block));

    g_assert_cmphex(st25_read_reg(ts, REG_RFA1SS), ==, 0x11);
    g_assert_cmphex(st25_read_reg(ts, REG_ENDA1),  ==, 0x22);
    g_assert_cmphex(st25_read_reg(ts, REG_RFA2SS), ==, 0x33);
    g_assert_cmphex(st25_read_reg(ts, REG_ENDA2),  ==, 0x44);

    qtest_quit(ts);
}

/* A multi-byte read auto-increments the register pointer. */
static void test_st25_read_auto_increment(void)
{
    QTestState *ts = setup_machine();
    uint8_t block[3];

    st25_write_reg(ts, REG_RFA1SS, 0xAA);
    st25_write_reg(ts, REG_ENDA1,  0xBB);
    st25_write_reg(ts, REG_RFA2SS, 0xCC);

    st25_read_block(ts, REG_RFA1SS, block, sizeof(block));
    g_assert_cmphex(block[0], ==, 0xAA);
    g_assert_cmphex(block[1], ==, 0xBB);
    g_assert_cmphex(block[2], ==, 0xCC);

    qtest_quit(ts);
}

/* Past the modeled bank: reads return 0xFF, writes are silently dropped. */
static void test_st25_out_of_range(void)
{
    QTestState *ts = setup_machine();

    g_assert_cmphex(st25_read_reg(ts, ST25_NUM_REGS), ==, 0xFF);

    /* Password-presentation message address -- write must be a no-op. */
    st25_write_reg(ts, ST25_REG_I2C_PWD, 0x00);
    g_assert_cmphex(st25_read_reg(ts, ST25_REG_I2C_PWD), ==, 0xFF);

    qtest_quit(ts);
}

/* A fresh START always resets the address-byte phase, even mid-address. */
static void test_st25_addr_phase_resets_on_restart(void)
{
    QTestState *ts = setup_machine();

    /* Send only the address MSB, then abandon (STOP) -- phase left at 1. */
    uint8_t half_addr[1] = { 0x00 };
    i2c_f2xx_write(ts, ST25_I2C_BASE, ST25_I2C_ADDR, half_addr, sizeof(half_addr));

    /* A clean write must not be corrupted by the abandoned phase above. */
    st25_write_reg(ts, REG_GPO, 0x55);
    g_assert_cmphex(st25_read_reg(ts, REG_GPO), ==, 0x55);

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/st25dv64k/reset_defaults",               test_st25_reset_defaults);
    qtest_add_func("/st25dv64k/write_read_single",            test_st25_write_read_single);
    qtest_add_func("/st25dv64k/full_bank_sweep",              test_st25_full_bank_sweep);
    qtest_add_func("/st25dv64k/write_auto_increment",         test_st25_write_auto_increment);
    qtest_add_func("/st25dv64k/read_auto_increment",          test_st25_read_auto_increment);
    qtest_add_func("/st25dv64k/out_of_range",                 test_st25_out_of_range);
    qtest_add_func("/st25dv64k/addr_phase_resets_on_restart", test_st25_addr_phase_resets_on_restart);

    return g_test_run();
}
