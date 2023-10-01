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

#include "../../hw/arm/prusa/stm32_chips/stm32f030xx.h"
#include "../../hw/arm/prusa/stm32_common/stm32_syscfg_regdata.h"

static void test_exticr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_SYSCFG].base_addr;
	int gpio_count = STM32_P_GPIO_END - STM32_P_GPIO_BEGIN;
	qtest_irq_intercept_out(global_qtest, "/machine/soc/SYSCFG");
	// Set all levels to 0 and verify the output signal is low.
	for (int i=0; i<gpio_count; i++)
	{
		for (int j=0; j<16; j++)
		qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, (16*i)+j, 0);
	}
	// Check they are low, then trip the rising edge trigger and confirm.
	for (int i=0; i<16; i++)
	{
		g_assert_false(qtest_get_irq(global_qtest, i));
		// Defaults are all GPIOA pins.
		qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, i, 1);
		g_assert_true(qtest_get_irq(global_qtest, i));
		// Reset for next test.
		qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, i, 0);
		g_assert_false(qtest_get_irq(global_qtest, i));
	}
	// Write each register with a different GPIO to confirm mappings.
	writel(STM32_RI_ADDRESS(base, RI_EXTICR1) , 0x3210);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR2) , 0x7654);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR3) , 0xBA98);
	writel(STM32_RI_ADDRESS(base, RI_EXTICR4) , 0xFEDC);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR1)), ==, 0x3210);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR2)), ==, 0x7654);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR3)), ==, 0xBA98);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_EXTICR4)), ==, 0xFEDC);

	for (int i=0; i<gpio_count; i++)
	{
		// Defaults are all GPIOA pins.
		qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, (i*16)+i, 1);
		g_assert_true(qtest_get_irq(global_qtest, i));
		// Reset for next test.
		qtest_set_irq_in(global_qtest, "/machine/soc/SYSCFG", NULL, (i*16)+i, 0);
		g_assert_false(qtest_get_irq(global_qtest, i));
	}
}

static void test_remap(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_SYSCFG].base_addr;
	writel(stm32f030xx_cfg.sram_base, 0xDEADBEEF);
	writel(stm32f030xx_cfg.flash_base, 0xBABECAFE);
	g_assert_cmphex(readl(stm32f030xx_cfg.sram_base), ==, 0xDEADBEEF);
	g_assert_cmphex(readl(0x0), ==, 0x0);
	writel(STM32_RI_ADDRESS(base, RI_MEMRMP_CFGR1) , MEMMODE_SRAM1);
	g_assert_cmphex(readl(0x0), ==, 0xDEADBEEF);
	writel(STM32_RI_ADDRESS(base, RI_MEMRMP_CFGR1) , MEMMODE_MAIN);
	g_assert_cmphex(readl(0x0), ==, 0x0);

}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();


    qtest_add_func("/stm32_syscfg/test_exticr", test_exticr);
    qtest_add_func("/stm32_syscfg/test_remap", test_remap);

    qtest_start("-machine stm32f030x4");

    ret = g_test_run();
    qtest_end();

    return ret;
}
