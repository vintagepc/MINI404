/*
 * QTest test for the fan implementation
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

#define QOM_PATH "/machine/peripheral/fan-P"
#define TEST_PREFIX "/mini404/parts/fan/"
#define MACHINE "prusa-mini"
#define FAN_MAX_RPM 6600

#define CMD_GET_RPM "fan::GetRPM()\n"
#define CMD_STALL "fan::Stall()\n"
#define CMD_RESUME "fan::Resume()\n"

static void send_scriptcmd(const char* cmd, int fd)
{
    g_assert_true(send(fd, cmd, strlen(cmd), 0) == strlen(cmd));
}

static void test_fan_rpm(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "rpm-out");

    g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 0);

    for (int i=0; i<256; i+=32)
    {
        qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, i);
        g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, (FAN_MAX_RPM*i)/255);
    }

    qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, 0);
    g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 0);

    qtest_quit(ts);
}

static void test_fan_tach(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "tach-out");

    g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 0);

    static const int tach_pwms[] = {64, 128, 192, 255};
    static const int tach_us_per_pulse[] = {9057, 4528, 3018, 2272};

    for (int i=0; i<ARRAY_SIZE(tach_pwms); i++)
    {
        qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, tach_pwms[i]);
        int64_t t_start = 0, t_end = 0;
        while(qtest_get_irq_level(ts, 0) == 1)
        {
            t_start = qtest_clock_step_next(ts);
        }
        t_start = qtest_clock_step(ts,0);
        while(qtest_get_irq_level(ts, 0) == 0)
        {
            t_end = qtest_clock_step_next(ts);
        }
        t_end = qtest_clock_step(ts,0);
        g_assert_cmpint((t_end - t_start)/1000, ==, tach_us_per_pulse[i]);
    }

    // check the tach stops when pwm is 0
    qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, 0);
    for (int i=0; i<10; i++)
    {
        qtest_clock_step_next(ts);
        g_assert_cmpint(qtest_get_irq_level(ts, 0), ==, 1);
    }

    qtest_quit(ts);
}

static void test_fan_scripting(void)
{
    /* Setup chardev */
    int fd = 0;
	QTestState *ts = qtest_init_with_serial("-machine " MACHINE " -global p404-scriptcon.input_id=s0 -global p404-scriptcon.no_echo=true", &fd);
    qtest_irq_intercept_out_named(ts, QOM_PATH, "tach-out");

    qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, 255);

    char buff[128] = {0};
    while(recv(fd, buff, sizeof(buff), MSG_DONTWAIT) > 0); // Clear the buffer;

    // Check fan RPM reporting:
    memset(buff, 0, sizeof(buff));
    send_scriptcmd(CMD_GET_RPM, fd);
    while(recv(fd, buff, sizeof(buff), MSG_DONTWAIT) == -1)
    {
        qtest_clock_step_next(ts);
    };
    g_assert_true(strncmp(buff, "6600", 4) == 0);

    qtest_set_irq_in(ts, QOM_PATH, "pwm-in", 0, 128);

    send_scriptcmd(CMD_GET_RPM, fd);
    while(recv(fd, buff, sizeof(buff), MSG_DONTWAIT) == -1)
    {
        qtest_clock_step_next(ts);
    };
    g_assert_true(strncmp(buff, "3312", 4) == 0);
    memset(buff, 0, sizeof(buff));

    // Wait for tach to be 1
    while(qtest_get_irq_level(ts, 0) != 1)
    {
        qtest_clock_step_next(ts);
    }

    // stall the fan.
    g_assert_true(qtest_get_irq_level(ts, 0) == 1);
    send_scriptcmd(CMD_STALL, fd);
    send_scriptcmd(CMD_GET_RPM, fd);
    while(recv(fd, buff, sizeof(buff), MSG_DONTWAIT) == -1)
    {
        qtest_clock_step_next(ts);
    };
    g_assert_true(qtest_get_irq_level(ts, 0) == 0);
    g_assert_true(strncmp(buff, "0", 1) == 0);

    send_scriptcmd(CMD_RESUME, fd);
    send_scriptcmd(CMD_GET_RPM, fd);
    while(recv(fd, buff, sizeof(buff), MSG_DONTWAIT) == -1)
    {
        qtest_clock_step_next(ts);
    };
    // g_assert_true(qtest_get_irq_level(ts, 0) == 1);
    g_assert_true(strncmp(buff, "3312", 4) == 0);


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
    qtest_add_func(TEST_PREFIX "rpm-out", test_fan_rpm);
    qtest_add_func(TEST_PREFIX "tach-out", test_fan_tach);
    qtest_add_func(TEST_PREFIX "script", test_fan_scripting);
    /* Run the tests */
    ret = g_test_run();

    return ret;
}
