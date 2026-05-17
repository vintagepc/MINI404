/*
 * MINI404 - LDC1612 2-Channel Inductance-to-Digital Converter Test
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
 * NOTE: This test requires a board model that instantiates the LDC1612 device.
 * Update MACHINE_NAME, LDC1612_I2C_BASE, and LDC1612_QOM_PATH when the board
 * model is created.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

/* TODO: update when the board model exists */
#define MACHINE_NAME        "prusa-clo-xx"

/* QOM path for the LDC1612 device (update to match the machine's peripheral ID) */
#define LDC1612_QOM_PATH    "/machine/peripheral/ldc1612"

/*
 * Base address of the STM32 I2C peripheral the LDC1612 is wired to.
 * Fill in the correct address for the target machine (e.g. 0x40005400 for I2C1).
 * NOTE: The STM32 common I2C model passes CR2.SADD directly to i2c_start_transfer,
 * so use the 7-bit address here, not the shifted 8-bit form.
 */
#define LDC1612_I2C_BASE    0x40005400ULL  /* TODO: confirm for target machine */

/* LDC1612 7-bit I2C address with ADDR pin low */
#define LDC1612_I2C_ADDR    0x2A

/* STM32 common I2C register offsets (stm32_i2c_regdata.h) */
#define I2C_CR1_OFF     0x00
#define I2C_CR2_OFF     0x04
#define I2C_ISR_OFF     0x18
#define I2C_RXDR_OFF    0x24
#define I2C_TXDR_OFF    0x28

/* CR2 field helpers */
#define CR2_SADD(a)     ((a) & 0x3FFU)
#define CR2_RD_WRN      (1U << 10)
#define CR2_START       (1U << 13)
#define CR2_STOP        (1U << 14)
#define CR2_NBYTES(n)   (((n) & 0xFFU) << 16)
#define CR2_AUTOEND     (1U << 25)

/* CR1 fields */
#define CR1_PE          (1U << 0)

/* ISR fields */
#define ISR_TXIS        (1U << 1)
#define ISR_RXNE        (1U << 2)
#define ISR_NACKF       (1U << 4)
#define ISR_TC          (1U << 6)

/* SD GPIO pin levels (active-low shutdown) */
#define SD_ASSERT       0   /* drive SD low  → shutdown */
#define SD_RELEASE      1   /* drive SD high → normal  */

/* LDC1612 register addresses */
#define REG_DATA_CH0        0x00
#define REG_DATA_CH0_LSB    0x01
#define REG_DATA_CH1        0x02
#define REG_DATA_CH1_LSB    0x03
#define REG_STATUS          0x18
#define REG_ERROR_CONFIG    0x19
#define REG_CONFIG          0x1A
#define REG_RESET_DEV       0x1C
#define REG_MANUFACTURER_ID 0x7E
#define REG_DEVICE_ID       0x7F

/* LDC1612 fixed ID values */
#define LDC1612_MFR_ID  0x5449U
#define LDC1612_DEV_ID  0x3055U

/* ERROR_CONFIG: enable DRDY to route to INTB */
#define ERROR_CONFIG_DRDY_2INT  (1U << 8)

/* CONFIG: disable INTB output */
#define CONFIG_INTB_DIS  (1U << 7)

/* STATUS: data ready */
#define STATUS_DRDY  (1U << 6)

/* RESET_DEV: assert device reset */
#define RESET_DEV_FLAG  (1U << 15)

/* A 28-bit inductance measurement value used throughout the tests */
#define TEST_CH0_VAL  0x0ABC1234U
#define TEST_CH1_VAL  0x00FEDCBAU

/* --------------------------------------------------------------------------
 * STM32 common I2C helpers
 *
 * Write path is functional in the current QEMU model (CR2 START → TXDR bytes).
 * Read path depends on RXNE being set after a read-direction START, which the
 * STM32 common I2C model does not yet implement.  The i2c_read_reg16 helper is
 * provided for completeness; the underlying model will need to be fixed before
 * read-based assertions in the tests below become operational.
 * -------------------------------------------------------------------------- */

/* Enable the peripheral before use */
static void i2c_enable(QTestState *ts)
{
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR1_OFF, CR1_PE);
}

/*
 * Write a 16-bit value to reg in one 3-byte I2C transaction:
 *   START + addr(W) + [reg, MSB, LSB] + STOP
 */
static void i2c_write_reg16(QTestState *ts, uint8_t dev_addr,
                             uint8_t reg, uint16_t value)
{
    uint32_t cr2 = CR2_SADD(dev_addr) | CR2_NBYTES(3) | CR2_AUTOEND | CR2_START;
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR2_OFF, cr2);
    /* Model sets TXIS immediately after START; write all three bytes */
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_TXDR_OFF, reg);
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_TXDR_OFF, (value >> 8) & 0xFF);
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_TXDR_OFF, value & 0xFF);
}

/*
 * Read a 16-bit value from reg using a write-then-repeated-start read.
 *
 * TODO: The STM32 common I2C model does not set RXNE after a read-direction
 * START, so the RXDR reads below will return stale data until that is fixed.
 * The correct model behavior should be:
 *   1. Write reg pointer (1 byte, no STOP)
 *   2. Repeated START in read direction → slave receives I2C_START_RECV
 *   3. Model calls i2c_recv(), stores result, sets RXNE
 *   4. Host reads RXDR to get each byte
 */
static uint16_t i2c_read_reg16(QTestState *ts, uint8_t dev_addr, uint8_t reg)
{
    /* Step 1: send register pointer (write, no STOP so TC is set not STOPF) */
    uint32_t cr2_wr = CR2_SADD(dev_addr) | CR2_NBYTES(1) | CR2_START;
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR2_OFF, cr2_wr);
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_TXDR_OFF, reg);

    /*
     * Step 2: prime RD_WRN in a separate write (workaround for the model
     * capturing is_read from the previous CR2 state, not the new write value).
     */
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR2_OFF,
                 CR2_SADD(dev_addr) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND);

    /* Step 3: repeated START in read direction */
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR2_OFF,
                 CR2_SADD(dev_addr) | CR2_RD_WRN | CR2_NBYTES(2) | CR2_AUTOEND | CR2_START);

    /* Step 4: read two bytes (MSB then LSB) */
    uint32_t msb = qtest_readl(ts, LDC1612_I2C_BASE + I2C_RXDR_OFF) & 0xFF;
    uint32_t lsb = qtest_readl(ts, LDC1612_I2C_BASE + I2C_RXDR_OFF) & 0xFF;
    return (uint16_t)((msb << 8) | lsb);
}

/* --------------------------------------------------------------------------
 * Test helpers
 * -------------------------------------------------------------------------- */

static QTestState *setup_machine(void)
{
    return qtest_init("-machine " MACHINE_NAME);
}

/*
 * Inject a 28-bit measurement value into the given channel (0 or 1) by
 * driving the named "ch" GPIO input on the LDC1612 device.
 */
static void inject_measurement(QTestState *ts, int ch, uint32_t val)
{
    qtest_set_irq_in(ts, LDC1612_QOM_PATH, "ch", ch, (int)val);
}

/* --------------------------------------------------------------------------
 * Tests
 * -------------------------------------------------------------------------- */

/*
 * Verify that the read-only ID registers return the correct fixed values.
 * Depends on the I2C read path being functional (see i2c_read_reg16 TODO).
 */
static void test_ldc1612_id_registers(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);

    g_assert_cmphex(i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_MANUFACTURER_ID),
                    ==, LDC1612_MFR_ID);
    g_assert_cmphex(i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DEVICE_ID),
                    ==, LDC1612_DEV_ID);

    qtest_quit(ts);
}

/*
 * With factory-default configuration (DRDY_2INT = 0), injecting data must
 * NOT assert the INTB output even though DRDY is set internally.
 */
static void test_ldc1612_no_interrupt_by_default(void)
{
    QTestState *ts = setup_machine();
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    /* INTB starts de-asserted (high) */
    g_assert_true(qtest_get_irq_level(ts, 0));

    inject_measurement(ts, 0, TEST_CH0_VAL);

    /* INTB must still be high: DRDY_2INT is 0 by default */
    g_assert_true(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/*
 * After enabling DRDY_2INT in ERROR_CONFIG, injecting data must assert INTB
 * (drive it low).  INTB_DIS defaults to 0 in CONFIG so no extra setup is
 * needed for the interrupt output pin.
 */
static void test_ldc1612_interrupt_on_data_ready(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    /* INTB starts de-asserted */
    g_assert_true(qtest_get_irq_level(ts, 0));

    /* Enable DRDY → INTB routing */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);

    /* Inject CH0 measurement */
    inject_measurement(ts, 0, TEST_CH0_VAL);

    /* INTB must now be asserted (low) */
    g_assert_false(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/*
 * Reading STATUS must clear the DRDY flag and de-assert INTB.
 * Depends on both the I2C write path (to enable DRDY_2INT) and the I2C
 * read path (to read STATUS and trigger side-effect clearing).
 */
static void test_ldc1612_status_read_clears_drdy(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);
    inject_measurement(ts, 0, TEST_CH0_VAL);
    g_assert_false(qtest_get_irq_level(ts, 0));  /* INTB asserted */

    /* Reading STATUS clears DRDY and de-asserts INTB */
    uint16_t status = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_STATUS);
    g_assert_cmphex(status & STATUS_DRDY, ==, STATUS_DRDY);  /* was set before read */

    /* TODO: verify INTB is de-asserted after STATUS read once I2C read path works:
     *   g_assert_true(qtest_get_irq_level(ts, 0));
     */

    qtest_quit(ts);
}

/*
 * Verify the INTB_DIS bit in CONFIG suppresses the interrupt even when DRDY
 * is set and DRDY_2INT is enabled.
 */
static void test_ldc1612_intb_disable(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);
    /* Disable the INTB output pin; keep other CONFIG bits at reset default */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_CONFIG, 0x2801 | CONFIG_INTB_DIS);

    inject_measurement(ts, 0, TEST_CH0_VAL);

    /* INTB must remain de-asserted (high) despite DRDY being set */
    g_assert_true(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/*
 * Verify injected measurement data for both channels is stored correctly in the
 * DATA registers.  A 28-bit value is split: upper 12 bits in DATA_CHx[11:0],
 * lower 16 bits in DATA_CHx_LSB.
 * Depends on the I2C read path being functional (see i2c_read_reg16 TODO).
 */
static void test_ldc1612_data_injection(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);

    inject_measurement(ts, 0, TEST_CH0_VAL);
    uint16_t msw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0);
    uint16_t lsw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0_LSB);
    g_assert_cmphex(msw, ==, (TEST_CH0_VAL >> 16) & 0x0FFF);
    g_assert_cmphex(lsw, ==, TEST_CH0_VAL & 0xFFFF);

    inject_measurement(ts, 1, TEST_CH1_VAL);
    msw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH1);
    lsw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH1_LSB);
    g_assert_cmphex(msw, ==, (TEST_CH1_VAL >> 16) & 0x0FFF);
    g_assert_cmphex(lsw, ==, TEST_CH1_VAL & 0xFFFF);

    qtest_quit(ts);
}

/*
 * Verify that writing to a read-only DATA register has no effect: the value
 * must remain whatever was previously injected via GPIO.
 * Depends on the I2C read path for verification.
 */
static void test_ldc1612_readonly_data_register(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);

    inject_measurement(ts, 0, TEST_CH0_VAL);

    /* Attempt to overwrite DATA_CH0 via I2C — should be silently ignored */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0, 0xDEAD);
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0_LSB, 0xBEEF);

    /* Value must be unchanged */
    uint16_t msw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0);
    uint16_t lsw = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DATA_CH0_LSB);
    g_assert_cmphex(msw, ==, (TEST_CH0_VAL >> 16) & 0x0FFF);
    g_assert_cmphex(lsw, ==, TEST_CH0_VAL & 0xFFFF);

    qtest_quit(ts);
}

/*
 * Write RESET_DEV with the RESET flag set and verify that ERROR_CONFIG returns
 * to its reset value (0x0000) and the fixed ID registers are preserved.
 * Depends on the I2C read path for ERROR_CONFIG and ID verification.
 */
static void test_ldc1612_software_reset(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);

    /* Write a non-default value to ERROR_CONFIG so we can observe the reset */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);

    /* Assert software reset */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_RESET_DEV, RESET_DEV_FLAG);

    /* ERROR_CONFIG must be cleared back to 0 */
    uint16_t ecfg = i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG);
    g_assert_cmphex(ecfg, ==, 0x0000);

    /* ID registers must still report the correct fixed values */
    g_assert_cmphex(i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_MANUFACTURER_ID),
                    ==, LDC1612_MFR_ID);
    g_assert_cmphex(i2c_read_reg16(ts, LDC1612_I2C_ADDR, REG_DEVICE_ID),
                    ==, LDC1612_DEV_ID);

    qtest_quit(ts);
}

/*
 * INTB must also de-assert after a software reset because DRDY is cleared and
 * ERROR_CONFIG.DRDY_2INT returns to 0.  This test exercises the interrupt path
 * without requiring I2C reads.
 */
static void test_ldc1612_reset_clears_interrupt(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);
    inject_measurement(ts, 0, TEST_CH0_VAL);
    g_assert_false(qtest_get_irq_level(ts, 0));  /* INTB asserted */

    /* Software reset must de-assert INTB */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_RESET_DEV, RESET_DEV_FLAG);
    g_assert_true(qtest_get_irq_level(ts, 0));   /* INTB de-asserted */

    qtest_quit(ts);
}

/*
 * While SD is asserted (low), data injection via the "ch" GPIO must be
 * silently ignored: DRDY stays clear and INTB stays de-asserted.
 */
static void test_ldc1612_shutdown_blocks_data_injection(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    /* Enable DRDY→INTB routing so we can observe DRDY indirectly */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);

    /* Assert SD (shutdown) */
    qtest_set_irq_in(ts, LDC1612_QOM_PATH, "sd", 0, SD_ASSERT);

    /* Data injection must be ignored while in shutdown */
    inject_measurement(ts, 0, TEST_CH0_VAL);
    inject_measurement(ts, 1, TEST_CH1_VAL);

    /* INTB must remain de-asserted (high) */
    g_assert_true(qtest_get_irq_level(ts, 0));

    qtest_quit(ts);
}

/*
 * While SD is asserted, I2C START must be NACKed: the STM32 I2C peripheral
 * sets NACKF in ISR when the addressed slave does not ACK.
 */
static void test_ldc1612_shutdown_nacks_i2c(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);

    /* Assert SD (shutdown) */
    qtest_set_irq_in(ts, LDC1612_QOM_PATH, "sd", 0, SD_ASSERT);

    /* Attempt a write — the START should be NACKed */
    uint32_t cr2 = CR2_SADD(LDC1612_I2C_ADDR) | CR2_NBYTES(1) | CR2_AUTOEND | CR2_START;
    qtest_writel(ts, LDC1612_I2C_BASE + I2C_CR2_OFF, cr2);

    g_assert_true(qtest_readl(ts, LDC1612_I2C_BASE + I2C_ISR_OFF) & ISR_NACKF);

    qtest_quit(ts);
}

/*
 * Releasing SD (driving it high) must reset the device to power-on defaults
 * and restore normal operation: subsequent data injection and interrupt
 * generation must work correctly.
 */
static void test_ldc1612_resume_from_shutdown(void)
{
    QTestState *ts = setup_machine();
    i2c_enable(ts);
    qtest_irq_intercept_out(ts, LDC1612_QOM_PATH);

    /* Configure DRDY→INTB, inject data, verify interrupt fires */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);
    inject_measurement(ts, 0, TEST_CH0_VAL);
    g_assert_false(qtest_get_irq_level(ts, 0));  /* INTB asserted */

    /* Shutdown: INTB must de-assert and registers must clear */
    qtest_set_irq_in(ts, LDC1612_QOM_PATH, "sd", 0, SD_ASSERT);
    g_assert_true(qtest_get_irq_level(ts, 0));   /* INTB de-asserted */

    /*
     * Release SD: device resets to power-on defaults (ERROR_CONFIG → 0).
     * INTB stays de-asserted since DRDY_2INT is now 0 again.
     */
    qtest_set_irq_in(ts, LDC1612_QOM_PATH, "sd", 0, SD_RELEASE);
    g_assert_true(qtest_get_irq_level(ts, 0));

    /* Re-enable DRDY→INTB and confirm new measurements are accepted */
    i2c_write_reg16(ts, LDC1612_I2C_ADDR, REG_ERROR_CONFIG, ERROR_CONFIG_DRDY_2INT);
    inject_measurement(ts, 1, TEST_CH1_VAL);
    g_assert_false(qtest_get_irq_level(ts, 0));  /* INTB asserted again */

    qtest_quit(ts);
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    /*
     * Tests marked "(GPIO only)" can run once the machine model exists,
     * regardless of the I2C read path status.
     * Tests marked "(I2C read)" additionally require the STM32 common I2C
     * model to implement the read path (see i2c_read_reg16 comment above).
     */
    qtest_add_func("/ldc1612/no_interrupt_by_default",         test_ldc1612_no_interrupt_by_default);         /* GPIO only */
    qtest_add_func("/ldc1612/interrupt_on_data_ready",         test_ldc1612_interrupt_on_data_ready);         /* I2C write + GPIO */
    qtest_add_func("/ldc1612/intb_disable",                    test_ldc1612_intb_disable);                    /* I2C write + GPIO */
    qtest_add_func("/ldc1612/reset_clears_interrupt",          test_ldc1612_reset_clears_interrupt);          /* I2C write + GPIO */
    qtest_add_func("/ldc1612/shutdown_blocks_data_injection",  test_ldc1612_shutdown_blocks_data_injection);  /* GPIO only */
    qtest_add_func("/ldc1612/shutdown_nacks_i2c",              test_ldc1612_shutdown_nacks_i2c);              /* I2C write + GPIO */
    qtest_add_func("/ldc1612/resume_from_shutdown",            test_ldc1612_resume_from_shutdown);            /* I2C write + GPIO */
    qtest_add_func("/ldc1612/id_registers",                    test_ldc1612_id_registers);                    /* I2C read */
    qtest_add_func("/ldc1612/data_injection",                  test_ldc1612_data_injection);                  /* I2C read */
    qtest_add_func("/ldc1612/readonly_data_register",          test_ldc1612_readonly_data_register);          /* I2C read */
    qtest_add_func("/ldc1612/software_reset",                  test_ldc1612_software_reset);                  /* I2C read */
    qtest_add_func("/ldc1612/status_read_clears_drdy",         test_ldc1612_status_read_clears_drdy);         /* I2C read */

    return g_test_run();
}
