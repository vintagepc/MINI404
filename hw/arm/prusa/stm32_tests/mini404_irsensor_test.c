/*
 * QTest test for the IR sensor
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

#define CMD_TOGGLE "ir-sensor::Toggle()\n"
#define CMD_SET "ir-sensor::Set(1)\n"
#define CMD_CLEAR "ir-sensor::Set(0)\n"

static void send_scriptcmd(const char* cmd, int fd)
{
    g_assert_true(send(fd, cmd, strlen(cmd), 0) == strlen(cmd));
}

static void test_reset(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine prusa-mini");
	qtest_irq_intercept_out(ts, "/machine/peripheral/ir-sensor");
    /* Assertions and other test logic */
    g_assert_false(qtest_get_irq(ts,0));

    // Toggle sensor
    qtest_hmp(ts, "sendkey f");
    qtest_clock_step_next(ts);

    g_assert_true(qtest_get_irq(ts,0));
    qtest_hmp(ts, "system_reset");
    qtest_clock_step_next(ts);
    // Did the sensor turn back off?
    g_assert_false(qtest_get_irq(ts,0));

    qtest_quit(ts);
}

static void test_key_toggle(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine prusa-mini");
	qtest_irq_intercept_out(ts, "/machine/peripheral/ir-sensor");
    /* Assertions and other test logic */
    g_assert_false(qtest_get_irq(ts,0));

    // Emulate a keypress
    qtest_hmp(ts, "sendkey f");
    qtest_clock_step_next(ts);

    g_assert_true(qtest_get_irq(ts,0));

    qtest_hmp(ts, "sendkey f");
    qtest_clock_step_next(ts);

    g_assert_false(qtest_get_irq(ts,0));

    qtest_quit(ts);
}

static void test_scripting(void)
{
    /* Setup chardev */
    int fd = 0;
	QTestState *ts = qtest_init_with_serial("-machine prusa-mini -global p404-scriptcon.input_id=s0", &fd);
	qtest_irq_intercept_out(ts, "/machine/peripheral/ir-sensor");
    /* Assertions and other test logic */
    g_assert_false(qtest_get_irq(ts,0));
    send_scriptcmd(CMD_TOGGLE, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_assert_true(qtest_get_irq(ts,0));
    // Turn it back off
    send_scriptcmd(CMD_TOGGLE, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_assert_false(qtest_get_irq(ts,0));
    // Now use Set
    send_scriptcmd(CMD_SET, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_assert_true(qtest_get_irq(ts,0));
    send_scriptcmd(CMD_SET, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_assert_true(qtest_get_irq(ts,0));
    // Now use clear
    send_scriptcmd(CMD_CLEAR, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_assert_false(qtest_get_irq(ts,0));

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
    qtest_add_func("/mini404/parts/irsensor/keypress", test_key_toggle);
    qtest_add_func("/mini404/parts/irsensor/scripting", test_scripting);
    qtest_add_func("/mini404/parts/irsensor/reset", test_reset);

    /* Run the tests */
    ret = g_test_run();

    return ret;
}
