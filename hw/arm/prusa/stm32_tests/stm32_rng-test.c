/*
 * QTest testcase for the STM32 RNG module.
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

#include "../../hw/arm/prusa/stm32_chips/stm32f407xx.h"
#include "../../hw/arm/prusa/stm32f407/stm32f4xx_rng_regdata.h"
#include "../../hw/arm/prusa/stm32_common/stm32_shared.h"

static void test_disabled(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_RNG].base_addr;

	// Check it's disabled because bit is cleared and the clock is off...
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x22);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR)), ==, 0x0);

	// Enable the clock and check the bit is clear but the interrupt is not.
	writel(STM32_RI_ADDRESS(stm32f407xx_cfg.perhipherals[STM32_P_RCC].base_addr, 13), 1U << 6);

	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x20);
	// Clear the IF
	writel(STM32_RI_ADDRESS(base, RI_SR), 0x20);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x00);

	// Now enable it:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x4);
	// Verify not DRDY:
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x00);
	// Verify no number;
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x0);

	// Now disable it:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x0);
}

static void test_get_number(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_RNG].base_addr;

	// First enable RNG
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x4);
	// Verify not DRDY:
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x00);
	// Verify no number;
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x0);
	// Wait for DRDY.
	int count = 0;
	while (count++ < 1000)
	{
        // This is tied to the default speed of the periph @16Mhz... can go away once
        // the RNG handles interrupt scheduling
		clock_step(2500);
		if (readl(STM32_RI_ADDRESS(base, RI_SR))&0x1)
		{
			break;
		}
	}
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR))&0x1, ==, 0x1);
	// See if we got something nonzero:
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), !=, 0x0);

	// Check that DRDY is now false again.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR))&0x1, ==, 0x0);
	// Disable
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x0);
}

static void test_readonly(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_RNG].base_addr;
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x0);
	writel(STM32_RI_ADDRESS(base, RI_DR), 0xDEADBEEF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x0);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

	qtest_start("-machine stm32f407xE");

    qtest_add_func("/stm32_rng/disabled", test_disabled);
    qtest_add_func("/stm32_rng/drdy", test_get_number);
    qtest_add_func("/stm32_rng/readonly_dr", test_readonly);

    ret = g_test_run();

	qtest_end();

    return ret;
}
