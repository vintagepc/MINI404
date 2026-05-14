/*
 * QTest test for the heater implementation, exercising the has_sock property.
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

#define QOM_PATH "/machine/peripheral/heater-E"
#define TEST_PREFIX "/mini404/parts/heater/"
#define MACHINE "prusa-mini"

/* prusa-mini extruder heater uses thermal_mass_x10=30, so thermalMass=3.0
 * with no sock and 3.0 * 0.85 = 2.55 with sock. The heater integrates
 * currentTemp += thermalMass * (pwm/255) * 0.25 every 250 ms tick.
 * Over 20 ticks at PWM=255:
 *   no-sock:  +15.0 °C  -> temp_out delta = 15.0 * 256 = 3840
 *   with-sock: +12.75 °C -> temp_out delta = 12.75 * 256 = 3264
 */
#define TICKS 20
#define TICK_NS (250ULL * 1000 * 1000)

static int32_t measure_rise(QTestState *ts, int ticks)
{
    qtest_irq_intercept_out_named(ts, QOM_PATH, "temp_out");
    qtest_set_irq_in(ts, QOM_PATH, "raw-pwm-in", 0, 255);
    /* First tick is scheduled at tNow+1 ms by heater_pwm_change. */
    qtest_clock_step(ts, 1ULL * 1000 * 1000);
    int32_t t0 = qtest_get_irq_level(ts, 0);
    for (int i = 0; i < ticks; i++) {
        qtest_clock_step(ts, TICK_NS);
    }
    int32_t tN = qtest_get_irq_level(ts, 0);
    return tN - t0;
}

static void test_heater_sock_off(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE
                                " -global heater.has_sock=off");
    int32_t rise = measure_rise(ts, TICKS);
    /* Expect ~3840 (15.0 °C * 256). Allow a generous tolerance band to
     * absorb floating-point rounding and any first-tick scheduling jitter. */
    g_assert_cmpint(rise, >, 3500);
    g_assert_cmpint(rise, <, 4200);
    qtest_quit(ts);
}

static void test_heater_sock_on(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE
                                " -global heater.has_sock=on");
    int32_t rise = measure_rise(ts, TICKS);
    /* Expect ~3264 (12.75 °C * 256). */
    g_assert_cmpint(rise, >, 3000);
    g_assert_cmpint(rise, <, 3500);
    qtest_quit(ts);
}

static void test_heater_sock_ratio(void)
{
    QTestState *ts_off = qtest_init("-machine " MACHINE
                                    " -global heater.has_sock=off");
    int32_t r_off = measure_rise(ts_off, TICKS);
    qtest_quit(ts_off);

    QTestState *ts_on = qtest_init("-machine " MACHINE
                                   " -global heater.has_sock=on");
    int32_t r_on = measure_rise(ts_on, TICKS);
    qtest_quit(ts_on);

    /* With-sock heatup must be strictly slower, and within ±3 % of the
     * HEATER_SOCK_MASS_FACTOR=0.85 ratio. */
    g_assert_cmpint(r_on, <, r_off);
    g_assert_cmpint(r_on * 100, >, r_off * 82);
    g_assert_cmpint(r_on * 100, <, r_off * 88);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func(TEST_PREFIX "sock-off", test_heater_sock_off);
    qtest_add_func(TEST_PREFIX "sock-on",  test_heater_sock_on);
    qtest_add_func(TEST_PREFIX "sock-ratio", test_heater_sock_ratio);

    return g_test_run();
}
