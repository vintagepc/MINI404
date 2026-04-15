/*
 * mini404_sn74cbt_test.c
 *
 * Copyright 2022 VintagePC <https://github.com/vintagepc/>
 * 
 * This file is part of Mini404.
 *
 * Description: Unit tests for SN74cbt component in the Prusa Mini404 board.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define QOM_PATH "/machine/peripheral/mux"
#define MACHINE "prusa-xl-extruder-040-0"

static void test_sn74cbt_output_enable(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out(ts, QOM_PATH);
    for (int i=0; i<4; i++) {
        // Output enable is disabled (active low)
        qtest_set_irq_in(ts, QOM_PATH, "nOE", 0, true);
        // Toggle one side and check the output:
        qtest_set_irq_in(ts, QOM_PATH, "B1", i, i + 100);
        g_assert_cmpint(qtest_get_irq_level(ts, i), !=, i + 100);

        qtest_set_irq_in(ts, QOM_PATH, "B1", i, i + 200);
        g_assert_cmpint(qtest_get_irq_level(ts, i), !=, i + 200);

        // now enable the output and verify the data is propagated
        qtest_set_irq_in(ts, QOM_PATH, "nOE", 0, false);
        g_assert_cmpint(qtest_get_irq_level(ts, i), ==, i + 200);

        qtest_set_irq_in(ts, QOM_PATH, "B1", i, i + 100);
        g_assert_cmpint(qtest_get_irq_level(ts, i), ==, i + 100);
    }
    qtest_quit(ts);
}

static void test_sn74cbt_select_input(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out(ts, QOM_PATH);
    // Output enable is disabled (active low)
    for (int i=0; i<4; i++) {
        qtest_set_irq_in(ts, QOM_PATH, "select", 0, true);
        // Toggle one side and check the output:
        qtest_set_irq_in(ts, QOM_PATH, "B1", i, i + 100);
        qtest_set_irq_in(ts, QOM_PATH, "B2", i, i + 200);
        g_assert_cmpint(qtest_get_irq_level(ts, i), ==, i + 200);

        qtest_set_irq_in(ts, QOM_PATH, "select", 0, false);
        g_assert_cmpint(qtest_get_irq_level(ts, i), ==, i + 100);
        qtest_set_irq_in(ts, QOM_PATH, "B1", i, i + 300);
        qtest_set_irq_in(ts, QOM_PATH, "B2", i, i + 400);

        g_assert_cmpint(qtest_get_irq_level(ts, i), ==, i + 300);
    }
    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/sn74cbt/output_enable", test_sn74cbt_output_enable);
    qtest_add_func("/sn74cbt/select_input", test_sn74cbt_select_input);
    return g_test_run();
}
