/*
 * QTest testcase for GPIO Hi-Z modeling on a shared fan-tach pin.
 *
 * Reproduces the false-fan-failure bug from PR #191: two fans share one
 * physical tach input pin through a hardware mux (PF13 select), and the
 * deselected fan must actually release the net instead of leaving it stuck.
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

#include "../stm32_common/stm32_gpio_regdata.h"

#define MACHINE "prusa-mk4-027c"
#define FAN_P "/machine/peripheral/fan-P"
#define FAN_E "/machine/peripheral/fan-E"

/* STM32F427, per stm32_chips/stm32f427xx.h */
#define GPIOE_BASE 0x40021000
#define GPIOF_BASE 0x40021400

#define GPIOE_TACH_PIN 10
#define GPIOF_SEL_PIN 13

static void test_tach_mux_switch_is_glitch_free(void)
{
    QTestState *ts = qtest_init("-machine " MACHINE);
    uint32_t gpioe = GPIOE_BASE;
    uint32_t gpiof = GPIOF_BASE;

    /* PF13 as push-pull output, driven high: selects the print fan. */
    qtest_writel(ts, STM32_RI_ADDRESS(gpiof, RI_MODER), 1U << (2*GPIOF_SEL_PIN));
    qtest_writel(ts, STM32_RI_ADDRESS(gpiof, RI_BSRR), 1U << GPIOF_SEL_PIN);

    qtest_set_irq_in(ts, FAN_P, "pwm-in", 0, 255);

    /* Wait for a real tach edge from the print fan through the mux. */
    int spins = 0;
    while (!(qtest_readl(ts, STM32_RI_ADDRESS(gpioe, RI_IDR)) & (1U << GPIOE_TACH_PIN))) {
        qtest_clock_step_next(ts);
        g_assert_cmpint(spins++, <, 100000);
    }

    /* Flip the mux to the heatbreak fan (idle, pwm=0) with no clock step in
     * between: the shared pin must reflect the new source immediately, not
     * the print fan's stale high level. */
    qtest_writel(ts, STM32_RI_ADDRESS(gpiof, RI_BRR), 1U << GPIOF_SEL_PIN);
    g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(gpioe, RI_IDR)) & (1U << GPIOE_TACH_PIN), ==, 0);

    /* It must stay low: the heatbreak fan is idle, not just transiently low. */
    for (int i = 0; i < 20; i++) {
        qtest_clock_step(ts, 1000);
        g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(gpioe, RI_IDR)) & (1U << GPIOE_TACH_PIN), ==, 0);
    }

    /* Switching back to the print fan resumes real tach edges immediately. */
    qtest_writel(ts, STM32_RI_ADDRESS(gpiof, RI_BSRR), 1U << GPIOF_SEL_PIN);
    spins = 0;
    bool saw_edge = false;
    while (spins++ < 100000) {
        qtest_clock_step_next(ts);
        if (qtest_readl(ts, STM32_RI_ADDRESS(gpioe, RI_IDR)) & (1U << GPIOE_TACH_PIN)) {
            saw_edge = true;
            break;
        }
    }
    g_assert_true(saw_edge);

    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    (void)stm32f030_gpio_reginfo;
    (void)stm32g070_c092_gpio_reginfo;
    (void)stm32f2xx_gpio_reginfo;
    (void)stm32f4xx_gpio_reginfo;
    (void)stm32h503_gpio_reginfo;

    qtest_add_func("/mini404/gpio_hiz/tach_mux_switch_is_glitch_free", test_tach_mux_switch_is_glitch_free);

    ret = g_test_run();

    return ret;
}
