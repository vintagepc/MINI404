/*
 * QTest testcase for the STM32 F4xx Flash Interface module.
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
#include "../../hw/arm/prusa/stm32f407/stm32f4xx_flashint_regdata.h"

#define KEY1 0x45670123UL
#define KEY2 0xCDEF89ABUL

static void test_lock(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_FINT].base_addr;
	REGDEF_NAME(flashif,cr) r;
	REGDEF_NAME(flashif,sr) s;
	// Check it's locked.
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);
	// Test lock bit cannot be cleared.
	writel(STM32_RI_ADDRESS(base, RI_CR),0x00000000);
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);

	// Check status:
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_SR)), ==, 0x00000000);

	r.SER = 1;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	r.STRT = 1;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	// Check an error occurred:
	s.raw = readl(STM32_RI_ADDRESS(base, RI_SR));
	g_assert_true(s.WRPERR);

	// Clear the error bit:
	writel(STM32_RI_ADDRESS(base, RI_SR),s.raw);
	s.raw = readl(STM32_RI_ADDRESS(base, RI_SR));
	g_assert_false(s.WRPERR);

	// Verify flash cannot be written:
	for (int i=0; i<stm32f407xx_cfg.flash_variants[0].mem_size; i+=4)
	{
		writel(stm32f407xx_cfg.flash_base + i, 0xDEADBEEF);
		g_assert_cmphex(readl(stm32f407xx_cfg.flash_base + i), == , 0);
	}
}

static void test_unlock(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_FINT].base_addr;

	REGDEF_NAME(flashif,cr) r;
	// Check it's locked.
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);

	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY1);
	// Check it's still locked.
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);
	writel(stm32f407xx_cfg.flash_base, 0xDEADBEEF);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0);

	//Write a random register before key2
	writel(STM32_RI_ADDRESS(base, RI_SR), 0x1);
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY2);

	// Check it's still locked.
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);
	writel(stm32f407xx_cfg.flash_base, 0xDEADBEEF);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0);

	// Try incorrect key2:
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY1);
	writel(STM32_RI_ADDRESS(base, RI_KEYR), 0xF00BA4);
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);
	writel(stm32f407xx_cfg.flash_base, 0xDEADBEEF);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0);

	// Now try proper unlock sequence:
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY1);
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY2);
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_false(r.LOCK);
	writel(stm32f407xx_cfg.flash_base, 0xDEADBEEF);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0xDEADBEEF);
	writel(stm32f407xx_cfg.flash_base, 0xCAFE);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0xCAFE);

	// Relock it and confirm it works:
	r.LOCK = true;
	writel(STM32_RI_ADDRESS(base, RI_CR),r.raw);
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_true(r.LOCK);
	writel(stm32f407xx_cfg.flash_base, 0xDEADBEEF);
	g_assert_cmphex(readl(stm32f407xx_cfg.flash_base), == , 0xCAFE);

}

static void test_erase(void)
{
	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_FINT].base_addr;

	REGDEF_NAME(flashif,cr) r;
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY1);
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY2);
	r.raw = readl(STM32_RI_ADDRESS(base, RI_CR));
	g_assert_false(r.LOCK);

	// Fill flash with something
	for (int i=0; i<stm32f407xx_cfg.flash_variants[0].mem_size; i+=4)
	{
		writel(stm32f407xx_cfg.flash_base + i, i);
	}

	r.SER = true;
	r.SNB = 1;
	writel(STM32_RI_ADDRESS(base, RI_CR),r.raw);
	// Make sure it hasn't erased yet... and that the previous write was good.
	for (int i=0; i<stm32f407xx_cfg.flash_variants[0].mem_size; i+=4)
	{
		g_assert_cmphex(readl(stm32f407xx_cfg.flash_base + i), == , i);
	}
	r.STRT = 1;
	writel(STM32_RI_ADDRESS(base, RI_CR),r.raw);
	// Check that only the selected page was erased.
	for (int i=0; i<stm32f407xx_cfg.flash_variants[0].mem_size; i+=4)
	{
		if (i < 16U*KiB || i >= 32U*KiB)
		{
			g_assert_cmphex(readl(stm32f407xx_cfg.flash_base + i), == , i);
		}
		else
		{
			g_assert_cmphex(readl(stm32f407xx_cfg.flash_base + i), == , 0xFFFFFFFF);
		}
	}
}


int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32f4xx_flashint/test_lock", test_lock);
    qtest_add_func("/stm32f4xx_flashint/test_unlock", test_unlock);
    qtest_add_func("/stm32f4xx_flashint/test_erase", test_erase);

    qtest_start("-machine stm32f407xE");

    ret = g_test_run();
    qtest_end();

    return ret;
}
