/*
 * QTest testcase for the STM32 CRC module.
 *
 * Copyright 2023 VintagePC <https://github.com/vintagepc>
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

#include "../stm32_chips/stm32f030xx.h"
#include "../stm32_common/stm32_iwdg_regdata.h"


static void test_lock(void)
{
	const uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_IWDG].base_addr;

	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RLR)), ==, 0xFFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_PR)), ==, 0x0);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_WINR)), ==, 0xFFF);

	// Check writes are not allowed if the IWDG is locked.
	writel(STM32_RI_ADDRESS(base, RI_PR) , 0xFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_PR)), ==, 0x0);

	writel(STM32_RI_ADDRESS(base, RI_WINR) , 0xF00);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_WINR)), ==, 0xFFF);

	writel(STM32_RI_ADDRESS(base, RI_RLR) , 0xBA4);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RLR)), ==, 0xFFF);

	// Unlock it and check again.
	writel(STM32_RI_ADDRESS(base, RI_KR) , 0x5555);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_KR)), ==, 0x0); // KR is write-only.
	writel(STM32_RI_ADDRESS(base, RI_PR) , 0xFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_PR)), ==, 0x7);

	writel(STM32_RI_ADDRESS(base, RI_WINR) , 0xFF00);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_WINR)), ==, 0xF00);

	writel(STM32_RI_ADDRESS(base, RI_RLR) , 0xFBA4);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RLR)), ==, 0xBA4);
	writel(STM32_RI_ADDRESS(base, RI_RLR) , 0xFFF); // restore.
	// Relock:
	writel(STM32_RI_ADDRESS(base, RI_KR) , 0x0);
	writel(STM32_RI_ADDRESS(base, RI_PR) , 0x0);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_PR)), ==, 0x7);

}

#define WD_STEPS(count,ps) (NANOSECONDS_PER_SECOND / 40000) * count * ps

// partially lifted from npcm tests.
static bool get_watchdog_action(QTestState *qts)
{
    QDict *ev = qtest_qmp_eventwait_ref(qts, "WATCHDOG");
    QDict *data;

    data = qdict_get_qdict(ev, "data");
    qobject_ref(data);
    qobject_unref(ev);
	bool did_reset = strcmp(qdict_get_str(data, "action"), "reset") == 0;
    qobject_unref(data);
	return did_reset;
}

static void test_prescale(void)
{
	const uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_IWDG].base_addr;
	uint8_t RLR[2] = {1, 5};
	// Enable the LSI clock for IWDG to function.
	for (int j=0; j<ARRAY_SIZE(RLR); j++)
		for (int i=0; i<ARRAY_SIZE(IWDG_PRESCALES); i++)
		{
			QTestState *ts = qtest_init("-machine stm32f030x4");
			qtest_writel(ts,  stm32f030xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x24, 0x1);
			qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0x5555);
			qtest_writel(ts, STM32_RI_ADDRESS(base, RI_PR) , i);
			qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RLR) , RLR[j]);
			qtest_clock_set(ts, 0);
			qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0xCCCC);
			qtest_clock_set(ts, WD_STEPS(RLR[j],IWDG_PRESCALES[i])-1);
			// Check that the system has not reset.
			g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), !=, 0xFFF);
			// No WD event...
			qtest_clock_step_next(ts);
			g_assert_true(get_watchdog_action(ts));
			qtest_qmp_eventwait(ts, "RESET");
			// Verify reset.
			g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), ==, 0xFFF);
			qtest_quit(ts);
		}
}

static void test_lsi_off(void)
{
	const uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_IWDG].base_addr;
	QTestState *ts = qtest_init("-machine stm32f030x4");
	// Check that the watchdog is off if LSI isn't enabled.
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0x5555);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_PR) , 0);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RLR) , 1);
	qtest_clock_set(ts, 0);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0xCCCC);
	qtest_clock_set(ts, 1000 * SCALE_US); // Wait 10x as long as we should - min is 100 uS.
	// Check that the system has not reset.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), !=, 0xFFF);
	// No WD event...
	qtest_quit(ts);
}

static void test_refresh(void)
{
	const uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_IWDG].base_addr;
	QTestState *ts = qtest_init("-machine stm32f030x4");
	qtest_writel(ts,  stm32f030xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x24, 0x1);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0x5555);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_PR) , 0);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RLR) , 1);
	qtest_clock_set(ts, 0);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0xCCCC);
	qtest_clock_set(ts, WD_STEPS(1,4)-1);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), !=, 0xFFF);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_KR) , 0xAAAA); // Tickle watchdog.
	qtest_clock_set(ts, (2*WD_STEPS(1,4))-2);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), !=, 0xFFF);
	qtest_clock_set(ts, (2*WD_STEPS(1,4))-1);
	g_assert_true(get_watchdog_action(ts));
	qtest_qmp_eventwait(ts, "RESET");
	// Verify reset.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RLR)), ==, 0xFFF);
	qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_iwdg/test_lock", test_lock);
    qtest_add_func("/stm32_iwdg/test_prescale", test_prescale);
    qtest_add_func("/stm32_iwdg/test_clock_off", test_lsi_off);
    qtest_add_func("/stm32_iwdg/test_refresh", test_refresh);

    qtest_start("-machine stm32f030x4");

    ret = g_test_run();
    qtest_end();

    return ret;
}
