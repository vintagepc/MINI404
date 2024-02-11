/*
 * mini404_sn74hc4052_test.c
 *
 * Copyright 2022 VintagePC <https://github.com/vintagepc/>
 * 
 * This file is part of Mini404.
 *
 * Description: Unit tests for SN74HC4052 component in the Prusa Mini404 board.
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
#define MACHINE "prusa-xl-bed-060"

static void test_sn74hc4052_reset(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out(ts, QOM_PATH);
    for (int bank = 0; bank < 2; bank++) {
        // Set up the inputs:
        for (int j = 0; j < 4; j++) {
            qtest_set_irq_in(ts, QOM_PATH, bank ? "2Y" : "1Y", j, j + (bank? 300 : 100));
        }
    }
    // Select line 2
    qtest_set_irq_in(ts, QOM_PATH, "select", 0, 0);
    qtest_set_irq_in(ts, QOM_PATH, "select", 1, 1);
    // Verify output is ok for the selected line
    g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 102);
    g_assert_cmpint(qtest_get_irq_level(ts, 1), ==, 302);

    // Reset the machine
    qtest_hmp(ts, "system_reset");
    // Redo the inputs to replace the machine's reset values. This should probably be a dedicated test machine sometime...
    for (int bank = 0; bank < 2; bank++) {
        // Set up the inputs:
        for (int j = 0; j < 4; j++) {
            qtest_set_irq_in(ts, QOM_PATH, bank ? "2Y" : "1Y", j, j + (bank? 300 : 100));
        }
    }

    // Reset should set the channel to start channel - for this machine, it's 1.
    g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 101);
    g_assert_cmpint(qtest_get_irq_level(ts, 1), ==, 301);
    qtest_quit(ts);

}

static void test_sn74hc4052_select_input(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out(ts, QOM_PATH);
    // Output enable is disabled (active low)
    for (int bank = 0; bank < 2; bank++) {
        for (int i = 0; i < 4; i++) {
            // Set up the inputs:
            for (int j = 0; j < 4; j++) {
                qtest_set_irq_in(ts, QOM_PATH, bank ? "2Y" : "1Y", j, j + (bank? 300 : 100));
            }
            // Setup select lines:
            qtest_set_irq_in(ts, QOM_PATH, "select", 0, i & 0b01);
            qtest_set_irq_in(ts, QOM_PATH, "select", 1, (i & 0b10) >> 1 );
            // Check the output matches the correct input line
            g_assert_cmpint(qtest_get_irq_level(ts, bank), ==, i + (bank? 300 : 100));

            // Now change the inputs and check it changes only if we update the currently selected line:
            for(int j = 0; j < 4; j++) {
                qtest_set_irq_in(ts, QOM_PATH, bank ? "2Y" : "1Y", j, j + 200);
            }
            g_assert_cmpint(qtest_get_irq_level(ts, bank), ==, i + 200);
        }
    }
    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/sn74hc4052/select_input", test_sn74hc4052_select_input);
    qtest_add_func("/sn74hc4052/reset", test_sn74hc4052_reset);
    return g_test_run();
}
