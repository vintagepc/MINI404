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
#include "../stm32_common/stm32_crc_regdata.h"

static void test_storage_idr(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_CRC].base_addr;
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x00);
    writel(STM32_RI_ADDRESS(base, RI_IDR) , 0xffffffff);
	// Verify the register only accepts 8 bits.
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0xFF);
	// Verify register read only allows full u32 reads.
	g_assert_cmphex(readw(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x00);
	g_assert_cmphex(readb(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0x00);

	// Verify that reset does not clear the IDR.
 	writel(STM32_RI_ADDRESS(base, RI_IDR) , 0xBA);
 	writel(STM32_RI_ADDRESS(base, RI_CR) , 0x01);
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0xBA);

	// Verify that only 32 bit writes are allowed:
	writew(STM32_RI_ADDRESS(base, RI_IDR) , 0xCC);
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0xBA);
	writeb(STM32_RI_ADDRESS(base, RI_IDR) , 0xCC);
    g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_IDR)), ==, 0xBA);
}

static void test_dr_reset(void)
{
	uint32_t base = stm32f030xx_cfg.perhipherals[STM32_P_CRC].base_addr;
	writel(STM32_RI_ADDRESS(stm32f030xx_cfg.perhipherals[STM32_P_RCC].base_addr, 5), 64); // Enable the CRC peripheral's clock
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0xFFFFFFFF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_INIT)), ==, 0xFFFFFFFF);
	writel(STM32_RI_ADDRESS(base, RI_DR) , 0xDEADBEEF);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x81da1a18);
	// Check that reset copies INIT to DR:
 	writel(STM32_RI_ADDRESS(base, RI_CR) , 0x01);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0xFFFFFFFF);

	// // Check smaller writes behave as expected:
	// writeb(STM32_RI_ADDRESS(base, RI_DR) , 0xDE);
	// writeb(STM32_RI_ADDRESS(base, RI_DR) , 0xAD);
	// writeb(STM32_RI_ADDRESS(base, RI_DR) , 0xBE);
	// writeb(STM32_RI_ADDRESS(base, RI_DR) , 0xEF);
	// g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0x81da1a18);

	// Verify that changing INIT works as expected:
	writel(STM32_RI_ADDRESS(base, RI_INIT) , 0xBABECAFE);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_INIT)), ==, 0xBABECAFE);
 	writel(STM32_RI_ADDRESS(base, RI_CR) , 0x01);
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0xBABECAFE);
	// Verify reset was cleared:
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR)), ==, 0x0);

}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

	(void)stm32f030_crc_reginfo;
	(void)stm32g070_crc_reginfo;
	(void)stm32f2xx_crc_reginfo;
	(void)stm32f4xx_crc_reginfo;

    qtest_add_func("/stm32_crc/storage_idr", test_storage_idr);
    qtest_add_func("/stm32_crc/dr_reset", test_dr_reset);

    qtest_start("-machine stm32f030x4");

    ret = g_test_run();
    qtest_end();

    return ret;
}
