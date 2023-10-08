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

#include "../../hw/arm/prusa/stm32_chips/stm32g070xx.h"
#include "../../hw/arm/prusa/stm32g070/stm32g070_exti_regdata.h"

static void test_exticr(void)
{
	int gpio_count = STM32G070_GPIO_END - STM32_P_GPIO_BEGIN;
	//Set all levels to 0 and verify the output signal is low.
	for (int i=0; i<gpio_count; i++)
	{
		for (int j=0; j<16; j++)
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, (16*i)+j, 0);
	}
	// Check they are low, then trip the triggers and confirm they do not fire.
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(global_qtest, i));
		// Defaults are all GPIOA pins.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_RISING);
		g_assert_false(qtest_get_irq(global_qtest, i));
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_FALLING);
		g_assert_false(qtest_get_irq(global_qtest, i));
		// Reset for next test.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, 0);
		g_assert_false(qtest_get_irq(global_qtest, i));
	}
}

static void test_rising_edge_trigger(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_EXTI].base_addr;
	// Default pin config is A, check they they behave properly.
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(global_qtest, i));
		// Defaults are all GPIOA pins - enable Rising edge trigger and verify.
		writel(STM32_RI_ADDRESS(base, RI_RTSR), 1U << i);
		writel(STM32_RI_ADDRESS(base, RI_FTSR), 0);
		qtest_irq_clear_pulses(global_qtest);
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_RISING);
		for (int j=0; j<16; j++)
			g_assert_cmpint(qtest_get_irq_pulsed(global_qtest, j), ==, i==j);

		// Check interrupt flag(s) are correct:
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 1U << i);
		// clear flag:
		writel(STM32_RI_ADDRESS(base, RI_RPR), 1U << i);

		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 0);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);

		qtest_irq_clear_pulses(global_qtest);
		// Ensure no fall triggers.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_FALLING);
		for (int j=0; j<16; j++)
			g_assert_false(qtest_get_irq_pulsed(global_qtest, j));

		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 0);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);
		// Reset for next test.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, 0);
	}
}

static void test_falling_edge_trigger(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_EXTI].base_addr;
	// Default pin config is A, check they they behave properly.
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(global_qtest, i));
		// Defaults are all GPIOA pins - enable Rising edge trigger and verify.
		writel(STM32_RI_ADDRESS(base, RI_RTSR), 0);
		writel(STM32_RI_ADDRESS(base, RI_FTSR), 1U << i);
		qtest_irq_clear_pulses(global_qtest);
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_FALLING);
		for (int j=0; j<16; j++)
			g_assert_cmpint(qtest_get_irq_pulsed(global_qtest, j), ==, i==j);

		// Check interrupt flag(s) are correct:
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 1U << i);
		// clear flag:
		writel(STM32_RI_ADDRESS(base, RI_FPR), 1U << i);

		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 0);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);

		qtest_irq_clear_pulses(global_qtest);
		// Ensure no fall triggers.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, EXTI_RISING);
		for (int j=0; j<16; j++)
			g_assert_false(qtest_get_irq_pulsed(global_qtest, j));

		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 0);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);
		// Reset for next test.
		qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, i, 0);
	}
}

static void test_sw_trigger(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_EXTI].base_addr;

	// Clear others, it should work in all cases:
	writel(STM32_RI_ADDRESS(base, RI_FTSR), 0);
	for (int i=0; i<16; i++)
	{
		qtest_irq_clear_pulses(global_qtest);
		writel(STM32_RI_ADDRESS(base, RI_RTSR), 0);
		writel(STM32_RI_ADDRESS(base, RI_SWIER), 1U << i);
		for (int j=0; j<16; j++)
		{
			g_assert_cmpint(qtest_get_irq_pulsed(global_qtest, j), ==, i==j);
		}
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 1U << i);
		writel(STM32_RI_ADDRESS(base, RI_RPR), 0xFFFF);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);
	}
	// test multiple pins at once
	qtest_irq_clear_pulses(global_qtest);
	writel(STM32_RI_ADDRESS(base, RI_SWIER), 0b111100001111);
	for (int j=0; j<16; j++)
	{
		g_assert_cmpint(qtest_get_irq_pulsed(global_qtest, j), ==, (bool)((1U <<j) &  0b111100001111));
	}
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 0b111100001111);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 0);
	writel(STM32_RI_ADDRESS(base, RI_RPR), 0xFFFF);
	qtest_irq_clear_pulses(global_qtest);
}

// Note we are just testing the mapping logic here, not the actual system wiring-
// therefore it allows/expects triggers from GPIOs that don't actually exist in the G070.
// I'm assuming this EXTI is used in other platforms and so limitations like that should be
// checked in an overall machine/soc check instead.
static void test_mapping(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_EXTI].base_addr;
	// Enable all rising/falling edges.
	writel(STM32_RI_ADDRESS(base, RI_RTSR), 0xFFFF);
	writel(STM32_RI_ADDRESS(base, RI_FTSR), 0xFFFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RTSR)), ==, 0xFFFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FTSR)), ==, 0xFFFF);
	// Setup each CR to map a different gpio bank.
	writel(STM32_RI_ADDRESS(base, RI_EXTICR1), 0x03020100);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR2), 0x07060504);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR3), 0x0B0A0908);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR4), 0x0F0E0D0C);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR1)), ==, 0x03020100);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR2)), ==, 0x07060504);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR3)), ==, 0x0B0A0908);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR4)), ==, 0x0F0E0D0C);

	int gpio_count = STM32G070_GPIO_END - STM32_P_GPIO_BEGIN;
	for (int i=0; i<gpio_count; i++)
	{
		// Fire all the pins for a given bank.
		for (int j=0; j<16; j++)
		{
			qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, (16*i)+j, EXTI_RISING);
		}
		// Check that only the correct bit was set for the current bank.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_RPR)), ==, 1U << i);
			for (int j=0; j<16; j++)
		{
			qtest_set_irq_in(global_qtest, "/machine/soc/EXTI", NULL, (16*i)+j, EXTI_FALLING);
		}
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_FPR)), ==, 1U << i);
		// Clear.
		writel(STM32_RI_ADDRESS(base, RI_RPR), 0xFFFF);
		writel(STM32_RI_ADDRESS(base, RI_FPR), 0xFFFF);

	}

}
	// /
	// // Write each register with a different GPIO to confirm mappings.
	// writel(STM32_RI_ADDRESS(base, RI_EXTICR1) , 0x3210);
	// writel(STM32_RI_ADDRESS(base, RI_EXTICR2) , 0x7654);
	// writel(STM32_RI_ADDRESS(base, RI_EXTICR3) , 0xBA98);
	// writel(STM32_RI_ADDRESS(base, RI_EXTICR4) , 0xFEDC);
	// g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR1)), ==, 0x3210);
	// g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR2)), ==, 0x7654);
	// g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR3)), ==, 0xBA98);
	// g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR4)), ==, 0xFEDC);

	// for (int i=0; i<gpio_count; i++)
	// {
	// 	// Defaults are all GPIOA pins.
	// 	qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, (i*16)+i, 1);
	// 	g_assert_true(qtest_get_irq(global_qtest, i));
	// 	// Reset for next test.
	// 	qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, (i*16)+i, 0);
	// 	g_assert_false(qtest_get_irq(global_qtest, i));
	// }


int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();
    qtest_start("-machine stm32g070xB");
	qtest_irq_intercept_out_named(global_qtest, "/machine/soc/EXTI", "sysbus-irq");

    qtest_add_func("/stm32_exti/test_disabled", test_exticr);
    qtest_add_func("/stm32_exti/test_rising_edge_trigger", test_rising_edge_trigger);
    qtest_add_func("/stm32_exti/test_falling_edge_trigger", test_falling_edge_trigger);
    qtest_add_func("/stm32_exti/test_sw_trigger", test_sw_trigger);
    qtest_add_func("/stm32_exti/test_mapping", test_mapping);


    ret = g_test_run();
    qtest_end();

    return ret;
}
