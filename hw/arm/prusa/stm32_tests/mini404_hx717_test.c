/*
 * MINI404 - HX717 Test
 *
 * This file is part of the MINI404 project, an open-source 3D printer simulator.
 * Copyright 2024 VintagePC <https://github.com/vintagepc>
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

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define QOM_PATH "/machine/peripheral/hx717"

#define CMD_SET "hx717::Set(5)\n"

static void send_scriptcmd(const char* cmd, int fd)
{
    g_assert_true(send(fd, cmd, strlen(cmd), 0) == strlen(cmd));
}

static uint32_t hx717_read_data(QTestState *ts)
{
    uint32_t data = 0;
    for (int i = 0; i < 24; i++) {
        qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
        uint32_t bit = qtest_get_irq_level(ts,0);
        data <<= 1;
        data |= bit;
        qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
    }
    return data;
}

static void test_hx717_data_reading(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");
    // Set the input data for the test
    qtest_irq_intercept_out(ts, QOM_PATH);
    qtest_set_irq_in(ts, QOM_PATH, "input", 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, "input", 1, 2);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // Default CHA gain is 128, so the data should be 128.
    g_assert_cmpint(hx717_read_data(ts), ==, 128);
    qtest_quit(ts);
}

static void test_hx717_channel_gain_setting(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");
    // Set the input data for the test
    qtest_irq_intercept_out(ts, QOM_PATH);
    qtest_set_irq_in(ts, QOM_PATH, "input", 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, "input", 1, 3);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // Default CHA gain is 128, so the data should be 128.
    g_assert_cmpint(hx717_read_data(ts), ==, 128);
    // Clock 2 more bits, this will change to CH B/Gain 64
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step_next(ts);

    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // Default CHB gain is 64, so the data should be 3*64 = 192.
    g_assert_cmpint(hx717_read_data(ts), ==, 192);

    // Clock 3 more bits, this will change to CH A/Gain 64
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step_next(ts);

    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // 2nd CHA gain is 64, so the data should be 1*64 = 64.
    g_assert_cmpint(hx717_read_data(ts), ==, 64);


    // Clock 4 more bits, this will change to CH B/Gain 8
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step_next(ts);

    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // 2nd CHB gain is 64, so the data should be 3*8 = 24.
    g_assert_cmpint(hx717_read_data(ts), ==, 24);

    qtest_quit(ts);
}

static void test_hx717_scripting(void)
{
    /* Setup chardev */
    int fd = 0;
	QTestState *ts = qtest_init_with_serial("-machine prusa-mk4-027c -global p404-scriptcon.input_id=s0", &fd);
    /* Assertions and other test logic */
     // Set the input data for the test
    qtest_irq_intercept_out(ts, QOM_PATH);
    qtest_set_irq_in(ts, QOM_PATH, "input", 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, "input", 1, 2);
    send_scriptcmd(CMD_SET, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    // Wait for a DRDY:
    while(qtest_get_irq_level(ts,0) == 1)
    {
        qtest_clock_step_next(ts);
    }
    // Default CHA gain is 128, so the data should be 5*128.
    g_assert_cmpint(hx717_read_data(ts), ==, 640);

    qtest_quit(ts);
}


int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/hx717/data_reading", test_hx717_data_reading);
    qtest_add_func("/hx717/channel_gain_setting", test_hx717_channel_gain_setting);
    qtest_add_func("/hx717/scripring", test_hx717_scripting);
    return g_test_run();
}
