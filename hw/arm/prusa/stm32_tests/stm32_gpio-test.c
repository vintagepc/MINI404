/*
 * QTest testcase for the STM32 GPIO module.
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
#include "../stm32_common/stm32_gpio_regdata.h"

static void test_output_odr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_GPIOA].base_addr;
	qtest_irq_intercept_out(global_qtest, "/machine/soc/GPIOA");
	// Check output IRQs respond.
    writel(STM32_RI_ADDRESS(base, RI_ODR) , 0xffffffff);
	for (int i=0; i<16; i++)
	{
		g_assert_true(qtest_get_irq(global_qtest, i));
	}
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ODR)), ==, 0xFFFF);
	writel(STM32_RI_ADDRESS(base, RI_ODR) , 0x0);
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(global_qtest, i));
	}
	// Make sure they are mapped correctly by bit.
	for (int i=0; i<16; i++)
	{
		writel(STM32_RI_ADDRESS(base, RI_ODR) , 1U << i);
		for (int j=0; j<16; j++)
		{
			if (j==i)
			{
				g_assert_true(qtest_get_irq(global_qtest, j));
			}
			else
			{
				g_assert_false(qtest_get_irq(global_qtest, j));
			}
		}
	}
}

static void test_output_bsrr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_GPIOA].base_addr;
	qtest_irq_intercept_out(global_qtest, "/machine/soc/GPIOA");
	 writel(STM32_RI_ADDRESS(base, RI_ODR) , 0x0);
	// Test the BS behaviour
	uint32_t state = 0;
	for (int i=0; i<16; i++)
	{
		writel(STM32_RI_ADDRESS(base, RI_BSRR) , 1U << i);
		state |= 1U << i;
		// Check ODR for proper update.
    	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ODR)), ==, state);
		// Register is write-only.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_BSRR)), ==, 0x00);
		for (int j=0; j<16; j++)
		{
			if (j<=i)
			{
				g_assert_true(qtest_get_irq(global_qtest, j));
			}
			else
			{
				g_assert_false(qtest_get_irq(global_qtest, j));
			}
		}
	}
	// Check BR behaviour.
	for (int i=0; i<16; i++)
	{
		writel(STM32_RI_ADDRESS(base, RI_BSRR) , 1U << (i+16));
		// Register is write-only.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_BSRR)), ==, 0x00);
		state &= ~(1U << i);
		// Check ODR sync
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ODR)), ==, state);
		// Verify IRQ state.
		for (int j=0; j<16; j++)
		{
			if (j<=i)
			{
				g_assert_false(qtest_get_irq(global_qtest, j));
			}
			else
			{
				g_assert_true(qtest_get_irq(global_qtest, j));
			}
		}
	}
	// Verify that BS takes priority over BR
	g_assert_false(qtest_get_irq(global_qtest, 0));
	writel(STM32_RI_ADDRESS(base, RI_BSRR) , 0x10001);
	g_assert_true(qtest_get_irq(global_qtest, 0));
	writel(STM32_RI_ADDRESS(base, RI_BSRR) , 0x10000);
	g_assert_false(qtest_get_irq(global_qtest, 0));
}

static void test_output_brr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_GPIOA].base_addr;
	qtest_irq_intercept_out(global_qtest, "/machine/soc/GPIOA");
	writel(STM32_RI_ADDRESS(base, RI_ODR) , 0xFFFF);
	// Check BR behaviour.
	uint32_t state = 0xFFFF;
	for (int i=0; i<16; i++)
	{
		writel(STM32_RI_ADDRESS(base, RI_BRR) , 1U << i);
		// Register is write-only.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_BRR)), ==, 0x00);
		state &= ~(1U << i);
		// Check ODR sync
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ODR)), ==, state);
		// Verify IRQ state.
		for (int j=0; j<16; j++)
		{
			if (j<=i)
			{
				g_assert_false(qtest_get_irq(global_qtest, j));
			}
			else
			{
				g_assert_true(qtest_get_irq(global_qtest, j));
			}
		}
	}
}

static void test_input_idr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_GPIOA].base_addr;
	// Check read-only.
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x0);
    writel(STM32_RI_ADDRESS(base, RI_IDR) , 0xffffffff);
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x0);
    writel(STM32_RI_ADDRESS(base, RI_IDR) , 0x0);
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x0);
	for (int i=0; i<16; i++)
	{
		qtest_set_irq_in(global_qtest, "/machine/soc/GPIOA", NULL, i, 1);
    	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 1U << i);
		qtest_set_irq_in(global_qtest, "/machine/soc/GPIOA", NULL, i, 0);
    	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0);
	}
}

static void test_input_exti(void)
{
	QTestState *ts = qtest_init("-machine stm32f030x4");
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_GPIOA].base_addr;
	qtest_irq_intercept_out_named(ts, "/machine/soc/GPIOA", "exti");
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IDR)), ==, 0xFFFF);
	// Evaluate EXTIs. They are single-shot IRQs that fire on transition.
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(ts, i));
		qtest_set_irq_in(ts, "/machine/soc/GPIOA", NULL, i, 1);
		qtest_set_irq_in(ts, "/machine/soc/GPIOA", NULL, i, 1);
		g_assert_false(qtest_get_irq(ts, i));
		qtest_set_irq_in(ts, "/machine/soc/GPIOA", NULL, i, 0);
		g_assert_cmpint(qtest_get_irq_level(ts, i), ==, EXTI_FALLING);
		// Check it doesn't change if there's a re-trigger.
		qtest_set_irq_in(ts, "/machine/soc/GPIOA", NULL, i, 1);
		g_assert_cmpint(qtest_get_irq_level(ts, i), ==, EXTI_RISING);
		qtest_set_irq_in(ts, "/machine/soc/GPIOA", NULL, i, 0);
		g_assert_cmpint(qtest_get_irq_level(ts, i), ==, EXTI_FALLING);
	}
	qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

	(void)stm32f030_gpio_reginfo;
	(void)stm32g070_gpio_reginfo;
	(void)stm32f2xx_gpio_reginfo;
	(void)stm32f4xx_gpio_reginfo;

    qtest_add_func("/stm32_gpio/output_odr", test_output_odr);
    qtest_add_func("/stm32_gpio/output_bsrr", test_output_bsrr);
    qtest_add_func("/stm32_gpio/output_brr", test_output_brr);
    qtest_add_func("/stm32_gpio/input_idr", test_input_idr);
    qtest_add_func("/stm32_gpio/input_exti", test_input_exti);

    qtest_start("-machine stm32f030x4");

    ret = g_test_run();
    qtest_end();

    return ret;
}
