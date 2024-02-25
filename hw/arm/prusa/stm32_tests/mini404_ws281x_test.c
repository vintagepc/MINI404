/*
 * QTest test for the ws281x implementation
 *
 * Copyright 2024 VintagePC <https://github.com/vintagepc>
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

#define QOM_PATH "/machine/peripheral/ws281x"
#define TEST_PREFIX "/mini404/parts/ws281x/"
#define MACHINE "prusa-xl-extruder-040-0"

#define T0H_NS 350
#define T1H_NS 700
#define T0L_NS 800
#define T1L_NS 600
#define RESET_NS 50000

static void send_colour(QTestState *ts, uint32_t colour)
{
    // Send the data:
    for (int i = 23; i >= 0; i--) {
        bool bit = (colour >> i) & 1;
        qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
        qtest_clock_step(ts, bit ? T1H_NS : T0H_NS);
        qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
        qtest_clock_step(ts, bit ? T1L_NS : T0L_NS);
    }

}

static void test_reset(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "colour");

    // Toggle GPIO input to send color value
    uint32_t colour = 0xBEEF05;
    // Data format is GBR, so output should be:
    #define COLOUR_GBR 0xEFBE05

    send_colour(ts,colour);
    // Check 24-bit output
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, COLOUR_GBR);

    send_colour(ts, 0xFFFFFF);
    // Check it hasn't updated, the LED is now in passthrough.
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, COLOUR_GBR);

    // Now delay and check reset:
    qtest_clock_step(ts, RESET_NS);

    send_colour(ts, 0xFFFFFF);
    // Colour should have updated again
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, 0xFFFFFF);

    qtest_quit(ts);
}

static void test_ws281x_color(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "colour");

    // Toggle GPIO input to send color value
    uint32_t colour = 0xBEEF05;
    // Data format is GBR, so output should be:
    #define COLOUR_GBR 0xEFBE05

    send_colour(ts,colour);
    // Check 24-bit output
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, COLOUR_GBR);

    send_colour(ts, 0xFFFFFF);
    // Check it hasn't updated, the LED is now in passthrough.
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, COLOUR_GBR);

    qtest_quit(ts);
}

static void test_ws281x_passthru(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out(ts, QOM_PATH);

    // Toggle GPIO input to send color value
    uint32_t colour = 0xBEEF05;
    // Data format is GBR, so output should be:
    #define COLOUR_GBR 0xEFBE05

    send_colour(ts,colour);
    // Check 24-bit output
    g_assert_cmphex(qtest_get_irq_level(ts, 0), !=, COLOUR_GBR);

    send_colour(ts, 0xFFFFFF);
    // Check it has updated, the LED is now in passthrough.
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, 0xFFFFFF);

    send_colour(ts, 0xABCDEF);
    // Check it ha updated, the LED is now in passthrough.
    g_assert_cmphex(qtest_get_irq_level(ts, 0), ==, 0xABCDEF);

    qtest_quit(ts);
}

/* Define the main function */
int main(int argc, char **argv)
{
    int ret;
    /* Initialize the QEMU test environment */
    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    /* Add your test case to the test suite */
    qtest_add_func(TEST_PREFIX "reset", test_reset);
    qtest_add_func(TEST_PREFIX "colour", test_ws281x_color);
    qtest_add_func(TEST_PREFIX "passthru", test_ws281x_passthru);

    /* Run the tests */
    ret = g_test_run();

    return ret;
}
