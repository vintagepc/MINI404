/*
 * QTest testcase for the AT21CSxx/PF13 shared open-drain bus, driven through
 * the actual GPIOF register interface (MODER/OTYPER/PUPDR/BSRR/BRR/IDR)
 * instead of the device's own QOM gpio lines, to prove the GPIO Hi-Z
 * resolver's wired-AND arbitration end to end.
 *
 * This mirrors mini404_at21csxx_test.c's bit-bang protocol helpers, but
 * "drive low"/"release" go through GPIOF's own registers as real firmware
 * would, and "read" comes back from GPIOF_IDR.
 *
 * Copyright 2026 VintagePC <https://github.com/vintagepc>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#include "../stm32_common/stm32_gpio_regdata.h"

/* STM32F427, per stm32_chips/stm32f427xx.h */
#define GPIOF_BASE 0x40021400
#define GPIOF_SEL_PIN 13

#define MICROS(x) (x * 1000)

#define LOW1_NS 1500
#define LOW0_NS 10000
#define BIT_TIME 25000

#define READ_NS 1000

#define WRITE_CMD 0xA0
#define READ_CMD 0xA1

static void pf13_set(QTestState *ts, bool high)
{
    if (high) {
        qtest_writel(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_BSRR), 1U << GPIOF_SEL_PIN);
    } else {
        qtest_writel(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_BRR), 1U << GPIOF_SEL_PIN);
    }
}

static bool pf13_get(QTestState *ts)
{
    return !!(qtest_readl(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_IDR)) & (1U << GPIOF_SEL_PIN));
}

static bool do_setup(QTestState *ts)
{
    /* Push-pull-capable output register, but open-drain type + pull-up:
     * "high" means release (Hi-Z, pulled up unless AT21 sinks it low). */
    qtest_writel(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_PUPDR), 1U << (2*GPIOF_SEL_PIN));
    qtest_writel(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_OTYPER), 1U << GPIOF_SEL_PIN);
    qtest_writel(ts, STM32_RI_ADDRESS(GPIOF_BASE, RI_MODER), 1U << (2*GPIOF_SEL_PIN));

    pf13_set(ts, true);
    pf13_set(ts, false);
    qtest_clock_step(ts, MICROS(150));
    pf13_set(ts, true);
    g_assert_cmpint(pf13_get(ts), ==, 1);
    // Check device ACK:
    qtest_clock_step(ts, MICROS(100));
    pf13_set(ts, false);
    qtest_clock_step(ts, MICROS(1) + 1);
    pf13_set(ts, true);
    qtest_clock_step(ts, MICROS(3));
    bool ack = !pf13_get(ts);
    qtest_clock_step(ts, MICROS(150));
    g_assert_cmpint(ack, ==, 1);
    return ack;
}

static bool read_bit(QTestState *ts)
{
    pf13_set(ts, false);
    qtest_clock_step(ts, READ_NS);
    pf13_set(ts, true);
    qtest_clock_step(ts, 800);
    bool bit = pf13_get(ts);
    qtest_clock_step(ts, BIT_TIME - READ_NS - 800);
    return bit;
}

static void send_bit(QTestState *ts, bool bit)
{
    pf13_set(ts, false);
    qtest_clock_step(ts, bit ? LOW1_NS : LOW0_NS);
    pf13_set(ts, true);
    qtest_clock_step(ts, BIT_TIME - (bit ? LOW1_NS : LOW0_NS));
}

static void send_start(QTestState *ts)
{
    pf13_set(ts, true);
    qtest_clock_step(ts, MICROS(500) + 1);
}

static void send_byte(QTestState *ts, uint8_t byte)
{
    for(uint8_t mask = 0x80; mask >0; mask >>= 1)
    {
        send_bit(ts, byte & mask);
    }
}

// Sends a byte and checks ACK
static bool write_byte(QTestState *ts, uint8_t byte)
{
    send_byte(ts, byte);
    return !read_bit(ts);
}

static uint8_t read_byte(QTestState *ts, bool send_ack)
{
    uint8_t result = 0;
    for (int i=7; i >= 0; i--)
    {
        bool bit = read_bit(ts);
        result |= bit << i;
    }
    send_bit(ts, !send_ack);

    return result;
}

static void fill_eeprom(QTestState* ts)
{
    g_assert_true(write_byte(ts, WRITE_CMD));
    g_assert_true(write_byte(ts, 0)); // set address pointer

    for (int i=0; i<128; i++)
    {
        g_assert_true(write_byte(ts, i+128));
    }
}

static void test_gpio_opendrain_loveboard_boot_read(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    send_start(ts);

    write_byte(ts, READ_CMD);

    g_assert_cmphex(read_byte(ts, true), ==, 0x02); // ver
    g_assert_cmphex(read_byte(ts, true), ==, 0x20); // size LSB
    g_assert_cmphex(read_byte(ts, true), ==, 0x00); // size MSB
    g_assert_cmphex(read_byte(ts, false), ==, 0x1F); // bomID

    qtest_quit(ts);
}

static void test_gpio_opendrain_loopback(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    fill_eeprom(ts);
    write_byte(ts, 0xFF);

    send_start(ts);

    g_assert_true(write_byte(ts, WRITE_CMD));
    g_assert_true(write_byte(ts, 0x00)); // set address pointer
    send_start(ts);
    write_byte(ts, READ_CMD);
    g_assert_cmphex(read_byte(ts, true), ==, 0xFF);

    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    (void)stm32f030_gpio_reginfo;
    (void)stm32g070_c092_gpio_reginfo;
    (void)stm32f2xx_gpio_reginfo;
    (void)stm32f4xx_gpio_reginfo;
    (void)stm32h503_gpio_reginfo;

    qtest_add_func("/mini404/gpio_opendrain/loveboard_boot_read", test_gpio_opendrain_loveboard_boot_read);
    qtest_add_func("/mini404/gpio_opendrain/loopback", test_gpio_opendrain_loopback);

    ret = g_test_run();

    return ret;
}
