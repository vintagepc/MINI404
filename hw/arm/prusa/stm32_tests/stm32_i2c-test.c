/*
 * QTest testcase for the STM32 common I2C peripheral.
 *
 * Covers the C092 (single combined IRQ) and H503 (split event/error IRQs)
 * variants.  Slave-dependent tests use the prusa-clo-xx board, which
 * instantiates a C092 SoC with an LDC1612 inductance sensor at I2C address
 * 0x2B on I2C1.  These tests are primarily intended to verify correct
 * controller behaviour after migration of the manual register structs to the
 * generated register definitions.
 *
 * Copyright 2026 VintagePC <https://github.com/vintagepc/>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "libqtest-single.h"

#include "../stm32_common/stm32_i2c_regdata.h"
#include "../stm32_common/stm32_shared.h"

/*
 * I2C1 base address — identical across C092, H503, F030, and G070.
 * See generated/stm32c092/Addresses.h: C092_I2C1_ADDR = 0x40005400.
 */
#define I2C1_BASE  0x40005400ULL

#define I2C_REG(ri)  STM32_RI_ADDRESS(I2C1_BASE, (ri))

/* CR1 bit positions */
#define CR1_PE      BIT(0)
#define CR1_TXIE    BIT(1)
#define CR1_RXIE    BIT(2)
#define CR1_NACKIE  BIT(4)
#define CR1_STOPIE  BIT(5)
#define CR1_TCIE    BIT(6)
#define CR1_ERRIE   BIT(7)

/* CR2 field helpers */
#define CR2_SADD(a)    ((uint32_t)(a) & 0x3FFU)
#define CR2_RD_WRN     BIT(10)
#define CR2_START      BIT(13)
#define CR2_NBYTES(n)  ((uint32_t)((n) & 0xFFU) << 16)
#define CR2_AUTOEND    BIT(25)

/* ISR flag bits (same positions as the corresponding CR1 enable bits) */
#define ISR_TXE    BIT(0)
#define ISR_TXIS   BIT(1)
#define ISR_RXNE   BIT(2)
#define ISR_NACKF  BIT(4)
#define ISR_STOPF  BIT(5)
#define ISR_TC     BIT(6)
#define ISR_TCR    BIT(7)

/* ICR write-1-to-clear bits */
#define ICR_NACKCF  BIT(4)
#define ICR_STOPCF  BIT(5)

/* LDC1612 I2C address and key register used in transaction tests */
#define SLAVE_ADDR      0x2B
#define LDC1612_MFR_ID  0x5449U
#define REG_MFR_ID      0x7E
#define REG_ERR_CFG     0x19

/* QOM device path used for IRQ interception */
#define I2C1_QOM  "/machine/soc/I2C1"

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

/*
 * Issue a two-phase I2C read: write the one-byte register pointer, then
 * issue a repeated-START in read direction and collect two bytes.
 *
 * The intermediate "prime RD_WRN" write (step 2) is a model-level workaround:
 * is_read is captured from the register state just before the new CR2 write
 * lands, so pre-loading RD_WRN=1 ensures the START inherits the correct
 * transfer direction.
 */
static uint16_t i2c_read16(QTestState *ts, uint8_t dev, uint8_t reg)
{
    /* Step 1 – send register pointer (write, no STOP) */
    qtest_writel(ts, I2C_REG(RI_CR2), CR2_SADD(dev) | CR2_NBYTES(1) | CR2_START);
    qtest_writel(ts, I2C_REG(RI_TXDR), reg);

    /* Step 2 – prime RD_WRN so the next START sees is_read = 1 */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(dev) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND);

    /* Step 3 – repeated START in read direction */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(dev) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND | CR2_START);

    uint32_t msb = qtest_readl(ts, I2C_REG(RI_RXDR)) & 0xFF;
    uint32_t lsb = qtest_readl(ts, I2C_REG(RI_RXDR)) & 0xFF;
    return (uint16_t)((msb << 8) | lsb);
}

/* --------------------------------------------------------------------------
 * Group 1 – C092 register behaviour (no slave required)
 * -------------------------------------------------------------------------- */

/* ISR resets to 0x1 (TXE=1); all other registers reset to 0. */
static void test_reset_values(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR1)),     ==, 0x00000000);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR2)),     ==, 0x00000000);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_TIMINGR)), ==, 0x00000000);
    /* ISR[0] = TXE must be set on reset per the reference manual */
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_TXE,   ==, ISR_TXE);
    /* No error or status flags asserted at reset */
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, 0);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_STOPF, ==, 0);

    qtest_quit(ts);
}

/* CR1.PE write/read-back. */
static void test_pe_readback(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR1)) & CR1_PE, ==, 0);
    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR1)) & CR1_PE, ==, CR1_PE);
    /* Clear PE */
    qtest_writel(ts, I2C_REG(RI_CR1), 0);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR1)) & CR1_PE, ==, 0);

    qtest_quit(ts);
}

/*
 * OAR1 and TIMINGR round-trip.  Verifies that config registers survive a
 * write/read cycle unchanged (important after switching to generated regs).
 */
static void test_config_register_readback(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");

    /* OAR1: OA1EN=1, 7-bit address 0x42 in bits [7:1] */
    const uint32_t oar1_val = BIT(15) | (0x42U << 1);
    qtest_writel(ts, I2C_REG(RI_OAR1), oar1_val);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_OAR1)) & 0x000087FFU,
                    ==, oar1_val & 0x000087FFU);

    /* TIMINGR: a typical 100 kHz configuration word */
    const uint32_t timingr_val = 0x10C0EAFFU;
    qtest_writel(ts, I2C_REG(RI_TIMINGR), timingr_val);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_TIMINGR)), ==, timingr_val);

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * Group 2 – C092 no-slave behaviour
 * -------------------------------------------------------------------------- */

/* START to an unoccupied address sets ISR.NACKF. */
static void test_nack_on_empty_bus(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(0x50) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, ISR_NACKF);

    qtest_quit(ts);
}

/* ICR.NACKCF write-1-to-clear clears ISR.NACKF. */
static void test_icr_clears_nackf(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(0x50) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, ISR_NACKF);

    qtest_writel(ts, I2C_REG(RI_ICR), ICR_NACKCF);

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, 0);

    qtest_quit(ts);
}

/*
 * NACKIE enabled: a NACK asserts the IRQ line; ICR.NACKCF clears it.
 * The sysbus-irq interceptor is installed before the START so it sees the
 * rising edge.
 */
static void test_nackie_fires_irq(void)
{
    QTestState *ts = qtest_init("-machine stm32c092xB");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_NACKIE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(0x50) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);

    /* NACKF set → NACKIE enabled → IRQ fires */
    g_assert_true(qtest_get_irq_level(ts, 0));

    /* Clearing NACKF via ICR must de-assert the IRQ */
    qtest_writel(ts, I2C_REG(RI_ICR), ICR_NACKCF);
    g_assert_false(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * Group 3 – Transaction tests (prusa-clo-xx, LDC1612 at address 0x2B)
 * -------------------------------------------------------------------------- */

/* A write to a slave that ACKs must leave NACKF clear. */
static void test_write_no_nack(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);

    /* 3-byte write: [reg, MSB, LSB] with AUTOEND */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_NBYTES(3) | CR2_START | CR2_AUTOEND);
    qtest_writel(ts, I2C_REG(RI_TXDR), REG_ERR_CFG);
    qtest_writel(ts, I2C_REG(RI_TXDR), 0x01);
    qtest_writel(ts, I2C_REG(RI_TXDR), 0x00);

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, 0);

    qtest_quit(ts);
}

/* AUTOEND: after the last byte the controller auto-issues STOP → ISR.STOPF. */
static void test_write_autoend_sets_stopf(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_NBYTES(3) | CR2_START | CR2_AUTOEND);
    qtest_writel(ts, I2C_REG(RI_TXDR), REG_ERR_CFG);
    qtest_writel(ts, I2C_REG(RI_TXDR), 0x00);
    qtest_writel(ts, I2C_REG(RI_TXDR), 0x00);

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_STOPF, ==, ISR_STOPF);

    qtest_quit(ts);
}

/*
 * Without AUTOEND, NBYTES reaching 0 sets ISR.TC.
 * With TCIE enabled the event IRQ asserts at that moment.
 */
static void test_tc_after_nbytes_done(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_TCIE);

    /* 1-byte write, no AUTOEND */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_NBYTES(1) | CR2_START);

    /* Before the byte is sent, TC is not yet set */
    g_assert_false(qtest_get_irq_level(ts, 0));

    /* Sending the byte decrements NBYTES to 0 → TC=1 → TCIE → IRQ */
    qtest_writel(ts, I2C_REG(RI_TXDR), REG_MFR_ID);
    g_assert_true(qtest_get_irq_level(ts, 0));
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_TC, ==, ISR_TC);

    qtest_quit(ts);
}

/* STOPIE: IRQ asserts when STOPF is set; ICR.STOPCF clears both. */
static void test_stopie_fires_irq(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_STOPIE);

    /* 1-byte write with AUTOEND → STOPF on completion */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);
    qtest_writel(ts, I2C_REG(RI_TXDR), REG_MFR_ID);

    g_assert_true(qtest_get_irq_level(ts, 0));

    /* ICR.STOPCF clears ISR.STOPF → IRQ de-asserts */
    qtest_writel(ts, I2C_REG(RI_ICR), ICR_STOPCF);
    g_assert_false(qtest_get_irq_level(ts, 0));
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_STOPF, ==, 0);

    qtest_quit(ts);
}

/*
 * Full read transaction: register pointer write then repeated-START in the
 * read direction.  Verifies that the two received bytes reconstruct the
 * LDC1612 manufacturer ID (0x5449).
 */
static void test_read_transaction(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE);

    g_assert_cmphex(i2c_read16(ts, SLAVE_ADDR, REG_MFR_ID), ==, LDC1612_MFR_ID);

    /* STOPF should be set after the read AUTOEND */
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_STOPF, ==, ISR_STOPF);

    qtest_quit(ts);
}

/*
 * RXNE is set immediately after a read-direction START (the model pre-fetches
 * the first byte from the slave synchronously).  With RXIE enabled the IRQ
 * fires at that moment, before the firmware reads RXDR.
 */
static void test_rxne_set_after_read_start(void)
{
    QTestState *ts = qtest_init("-machine prusa-clo-xx");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_RXIE);

    /* Write the register pointer */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_NBYTES(1) | CR2_START);
    qtest_writel(ts, I2C_REG(RI_TXDR), REG_MFR_ID);

    /* Prime RD_WRN then start read */
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(SLAVE_ADDR) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND | CR2_START);

    /* RXNE is set synchronously; with RXIE enabled the IRQ fires immediately */
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_RXNE, ==, ISR_RXNE);
    g_assert_true(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * Group 4 – H503 split event/error IRQ
 * -------------------------------------------------------------------------- */

/*
 * The H503 exposes two separate IRQ lines: sysbus-irq[0] = event,
 * sysbus-irq[1] = error.  NACKF is classified as an event condition (not an
 * error), so it must fire the event IRQ and leave the error IRQ silent.
 */
static void test_h503_nackf_event_not_error(void)
{
    QTestState *ts = qtest_init("-machine stm32h503xx");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_NACKIE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(0x50) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);

    /* Event IRQ (index 0) fires for NACKF */
    g_assert_true(qtest_get_irq_level(ts, 0));
    /* Error IRQ (index 1) must stay silent — NACKF is not in IRQ_ERR */
    g_assert_false(qtest_get_irq_level(ts, 1));

    qtest_writel(ts, I2C_REG(RI_ICR), ICR_NACKCF);
    g_assert_false(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/*
 * ERRIE does not affect the event IRQ.  With NACKIE=0 and ERRIE=1 a NACK
 * must leave both IRQ lines silent (NACKF is not routed to the error IRQ).
 */
static void test_h503_errie_does_not_route_nackf(void)
{
    QTestState *ts = qtest_init("-machine stm32h503xx");
    qtest_irq_intercept_out_named(ts, I2C1_QOM, "sysbus-irq");

    /* Only ERRIE enabled — no event interrupt enables */
    qtest_writel(ts, I2C_REG(RI_CR1), CR1_PE | CR1_ERRIE);
    qtest_writel(ts, I2C_REG(RI_CR2),
                 CR2_SADD(0x50) | CR2_NBYTES(1) | CR2_START | CR2_AUTOEND);

    /* NACKF must be set in ISR */
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, ISR_NACKF);
    /* Neither IRQ line should fire */
    g_assert_false(qtest_get_irq_level(ts, 0));
    g_assert_false(qtest_get_irq_level(ts, 1));

    qtest_quit(ts);
}

/* H503 ISR reset value matches C092 (both generated from the same spec). */
static void test_h503_reset_values(void)
{
    QTestState *ts = qtest_init("-machine stm32h503xx");

    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_CR1)),     ==, 0x00000000);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_TXE,   ==, ISR_TXE);
    g_assert_cmphex(qtest_readl(ts, I2C_REG(RI_ISR)) & ISR_NACKF, ==, 0);

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    /* Group 1: register behaviour */
    qtest_add_func("/stm32_i2c/reset_values",              test_reset_values);
    qtest_add_func("/stm32_i2c/pe_readback",               test_pe_readback);
    qtest_add_func("/stm32_i2c/config_register_readback",  test_config_register_readback);

    /* Group 2: no-slave behaviour */
    qtest_add_func("/stm32_i2c/nack_on_empty_bus",         test_nack_on_empty_bus);
    qtest_add_func("/stm32_i2c/icr_clears_nackf",          test_icr_clears_nackf);
    qtest_add_func("/stm32_i2c/nackie_fires_irq",          test_nackie_fires_irq);

    /* Group 3: transaction tests (requires LDC1612 slave on prusa-clo-xx) */
    qtest_add_func("/stm32_i2c/write_no_nack",             test_write_no_nack);
    qtest_add_func("/stm32_i2c/write_autoend_sets_stopf",  test_write_autoend_sets_stopf);
    qtest_add_func("/stm32_i2c/tc_after_nbytes_done",      test_tc_after_nbytes_done);
    qtest_add_func("/stm32_i2c/stopie_fires_irq",          test_stopie_fires_irq);
    qtest_add_func("/stm32_i2c/read_transaction",          test_read_transaction);
    qtest_add_func("/stm32_i2c/rxne_set_after_read_start", test_rxne_set_after_read_start);

    /* Group 4: H503 split event/error IRQ */
    qtest_add_func("/stm32_i2c/h503_nackf_event_not_error",      test_h503_nackf_event_not_error);
    qtest_add_func("/stm32_i2c/h503_errie_does_not_route_nackf", test_h503_errie_does_not_route_nackf);
    qtest_add_func("/stm32_i2c/h503_reset_values",               test_h503_reset_values);

    ret = g_test_run();
    return ret;
}
