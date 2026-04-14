/*
 * QTest test for the MMU bridge
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
#include "../parts/mmu_bridge.h"

#define QOM_PATH "/machine/peripheral/mmu-bridge"

static void send_cmd(const char cmd, int fd)
{
    g_assert_true(send(fd, &cmd, 1, 0) == 1);
}

static void test_reset(void)
{
    /* Setup chardev */
    int fd = 0;
    char buff[10] = {0};
	QTestState *ts = qtest_init_with_serial("-machine prusa-mk4-027c-mmu -kernel /dev/zero -global mmu-bridge.input_id=s0", &fd);

    // Process the initial boot-up reset
    g_assert_cmpint(recv(fd, buff, sizeof(buff), MSG_DONTWAIT), ==, 1);
    g_assert_cmpint(buff[0], ==, RESET);

    // Check no input yet
    g_assert_cmpint(recv(fd, buff, sizeof(buff), MSG_DONTWAIT), ==, -1);
    
    // Fire reset
    qtest_set_irq_in(ts, QOM_PATH, "reset-in", 0, 1);
    qtest_clock_step_next(ts);
    // Did we receive the reset character?
    g_assert_true(recv(fd, buff, 1, 0) == 1);
    g_assert_cmphex(buff[0], ==, RESET);

    qtest_quit(ts);
}

static void test_fsensor(void)
{
    int fd = 0;
	QTestState *ts = qtest_init_with_serial("-machine prusa-mk4-027c-mmu -kernel /dev/zero -global mmu-bridge.input_id=s0", &fd);

    qtest_irq_intercept_out_named(ts, QOM_PATH, "fs-out");

    // Check IRQ is low
    g_assert_false(qtest_get_irq(ts,0));

    // Send a message
    send_cmd(FS_AUTO_SET, fd);
    qtest_clock_step_next(ts);
    g_assert_true(qtest_get_irq(ts,0));

    // Clear sensor
    send_cmd(FS_AUTO_CLEAR, fd);
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
    qtest_add_func("/mini404/parts/mmu-bridge/fsensor", test_fsensor);
    qtest_add_func("/mini404/parts/mmu-bridge/reset", test_reset);

    /* Run the tests */
    ret = g_test_run();

    return ret;
}
