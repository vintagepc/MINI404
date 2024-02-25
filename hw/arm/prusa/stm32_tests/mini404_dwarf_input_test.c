/* 
    mini404_dwarf_input_test.c - Test cases for the dwarf input peripheral.

    Copyright (C) 2024 VintagePC <https://github.com/vintagepc>

    This file is part of Mini404.

    Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define QOM_PATH "/machine/peripheral/dwarf-input"
#define TEST_PREFIX "/mini404/parts/dward-input/"
#define MACHINE "prusa-xl-extruder-040-0"

static void test_key_control(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine " MACHINE);
	qtest_irq_intercept_out(ts, QOM_PATH);
    /* Assertions and other test logic */

    g_free(qtest_hmp(ts, "system_reset"));

    // Check start state
    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));

    g_free(qtest_hmp(ts, "sendkey w"));
    g_assert_false(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));

    qtest_clock_step(ts, 101 * 1E6); // 100ms default hold-time
    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));

    g_free(qtest_hmp(ts, "sendkey s"));

    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_false(qtest_get_irq(ts, 1));

    qtest_clock_step(ts, 100 * 1E6); // 100ms default hold-time

    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));

    qtest_quit(ts);
}

static void test_reset(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine " MACHINE);
	qtest_irq_intercept_out(ts, QOM_PATH);
    /* Assertions and other test logic */

    g_free(qtest_hmp(ts, "sendkey w"));
    g_assert_false(qtest_get_irq(ts, 0));

    g_free(qtest_hmp(ts, "system_reset"));

    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));
    // Apparently HMP can't send multiple keys at once, delay until it's cleared.
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    g_free(qtest_hmp(ts, "sendkey s"));

    g_assert_false(qtest_get_irq(ts, 1));

    g_free(qtest_hmp(ts, "system_reset"));

    g_assert_true(qtest_get_irq(ts, 1));

    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    /* Add your test case to the test suite */
    qtest_add_func(TEST_PREFIX "keys", test_key_control);
    qtest_add_func(TEST_PREFIX "reset", test_reset);

    /* Run the tests */
    return g_test_run();
}
