/* 
    mini404_encoder_test.c - Test cases for the encoder input peripheral.

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

#define QOM_PATH "/machine/peripheral/encoder-input"
#define MACHINE "prusa-mini"
#define CMD_TWUP "encoder-input::Twist(1)\n"
#define CMD_TWDOWN "encoder-input::Twist(-1)\n"
#define CMD_PUSH "encoder-input::Push()\n"
#define CMD_RESET "encoder-input::Reset()\n"

static void send_scriptcmd(const char* cmd, int fd)
{
    g_assert_true(send(fd, cmd, strlen(cmd), 0) == strlen(cmd));
}

static void test_key_twist(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine " MACHINE);
	qtest_irq_intercept_out_named(ts, QOM_PATH, "encoder-ab");
    /* Assertions and other test logic */

    // Emulate a keypress
    g_free(qtest_hmp(ts, "sendkey s"));
    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Emulate a keypress
    g_free(qtest_hmp(ts, "sendkey s"));
    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // Now the other direction:
    g_free(qtest_hmp(ts, "sendkey w"));
    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Now the other direction:
    g_free(qtest_hmp(ts, "sendkey w"));
    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // ensure no further changes.
    qtest_clock_step(ts, 1000U*1E6); // 1s

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_quit(ts);
}


static void test_key_push(void)
{
    /* Test code goes here */
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "encoder-button");

    // Emulate a keypress
    g_free(qtest_hmp(ts, "sendkey ret"));

    g_assert_false(qtest_get_irq(ts,0));

    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));

    // no further changes.
    qtest_clock_step(ts, 100U*1E6); // 100ms
    g_assert_true(qtest_get_irq(ts,0));

    qtest_quit(ts);
}

static void test_mouse_push(void)
{
    /* Test code goes here */
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "touch");

    // Emulate a keypress
    g_free(qtest_hmp(ts, "mouse_button 1"));
    g_assert_true(qtest_get_irq(ts,0));

    g_free(qtest_hmp(ts, "mouse_button 0"));
    g_assert_false(qtest_get_irq(ts,0));

    // no further changes.
    qtest_clock_step(ts, 100U*1E6); // 100ms
    g_assert_false(qtest_get_irq(ts,0));

    qtest_quit(ts);
}

static void test_mouse_twist(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine " MACHINE);
	qtest_irq_intercept_out_named(ts, QOM_PATH, "encoder-ab");
    /* Assertions and other test logic */

    // Emulate a keypress
    g_free(qtest_hmp(ts, "mouse_move 0 0 -1"));
    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Emulate a keypress
    g_free(qtest_hmp(ts, "mouse_move 0 0 -1"));
    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // Now the other direction:
    g_free(qtest_hmp(ts,  "mouse_move 0 0 1"));
    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Now the other direction:
    g_free(qtest_hmp(ts,  "mouse_move 0 0 1"));
    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // ensure no further changes.
    qtest_clock_step(ts, 1000U*1E6); // 1s

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_quit(ts);
}


static void test_mouse_xy(void)
{
    /* Test code goes here */
	QTestState *ts = qtest_init("-machine " MACHINE);
	qtest_irq_intercept_out_named(ts, QOM_PATH, "cursor_xy");
    /* Assertions and other test logic */

    // Emulate a mouse move
    g_free(qtest_hmp(ts, "mouse_move 100 250"));

    g_assert_cmpint(qtest_get_irq_level(ts,0), ==, 100);
    g_assert_cmpint(qtest_get_irq_level(ts,1), ==, 250);

    // Emulate a mouse move
    g_free(qtest_hmp(ts, "mouse_move 200 150"));

    g_assert_cmpint(qtest_get_irq_level(ts,0), ==, 200);
    g_assert_cmpint(qtest_get_irq_level(ts,1), ==, 150);

    // Emulate a mouse move
    g_free(qtest_hmp(ts, "mouse_move 300 350"));

    g_assert_cmpint(qtest_get_irq_level(ts,0), ==, 300);
    g_assert_cmpint(qtest_get_irq_level(ts,1), ==, 350);

    qtest_quit(ts);
}


static void test_script_push(void)
{
    /* Test code goes here */
    int fd = 0;
    QTestState *ts = qtest_init_with_serial("-machine " MACHINE " -global p404-scriptcon.input_id=s0", &fd);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "encoder-button");

    // Emulate a keypress
    send_scriptcmd(CMD_PUSH, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);

    g_assert_false(qtest_get_irq(ts,0));

    qtest_clock_step(ts, 100U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));

    // no further changes.
    qtest_clock_step(ts, 100U*1E6); // 100ms
    g_assert_true(qtest_get_irq(ts,0));

    qtest_quit(ts);
}

static void test_script_twist(void)
{
    /* Test code goes here */
    int fd = 0;
	QTestState *ts = qtest_init_with_serial("-machine " MACHINE " -global p404-scriptcon.input_id=s0", &fd);
	qtest_irq_intercept_out_named(ts, QOM_PATH, "encoder-ab");
    /* Assertions and other test logic */

    g_free(qtest_hmp(ts, "system_reset"));

    // Emulate a keypress
    send_scriptcmd(CMD_TWDOWN, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step(ts, 101U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Emulate a keypress
    send_scriptcmd(CMD_TWDOWN, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step(ts, 105U*1E6); // 100ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // Now the other direction:
    send_scriptcmd(CMD_TWUP, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step(ts, 105U*1E6); // 100ms
    g_assert_false(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_false(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    // Now the other direction:
    send_scriptcmd(CMD_TWUP, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    qtest_clock_step(ts, 101U*1E6); // 100ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_false(qtest_get_irq(ts,1));

    qtest_clock_step(ts, 10U*1E6); // 10ms

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    // ensure no further changes.
    qtest_clock_step(ts, 1000U*1E6); // 1s

    g_assert_true(qtest_get_irq(ts,0));
    g_assert_true(qtest_get_irq(ts,1));

    qtest_quit(ts);
}

static void test_script_reset(void)
{
    /* Test code goes here */
    int fd = 0;
    QTestState *ts = qtest_init_with_serial("-machine " MACHINE " -global p404-scriptcon.input_id=s0", &fd);

    // Emulate a keypress
    send_scriptcmd(CMD_RESET, fd);
    qtest_clock_step_next(ts);
    qtest_clock_step_next(ts);
    
    qtest_qmp_eventwait(ts, "RESET");
    
    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    /* Add your test case to the test suite */
    qtest_add_func("/mini404/parts/encoder/keytwist", test_key_twist);
    qtest_add_func("/mini404/parts/encoder/keypush", test_key_push);
    qtest_add_func("/mini404/parts/encoder/mousepush", test_mouse_push);
    qtest_add_func("/mini404/parts/encoder/mousetwist", test_mouse_twist);
    qtest_add_func("/mini404/parts/encoder/mousexy", test_mouse_xy);
    qtest_add_func("/mini404/parts/encoder/scripttwist", test_script_twist);
    qtest_add_func("/mini404/parts/encoder/scriptpush", test_script_push);
    qtest_add_func("/mini404/parts/encoder/scriptreset", test_script_reset);

    /* Run the tests */
    return g_test_run();
}
