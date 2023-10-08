/*
 * QTest testcase for the STM32 DBG module.
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
#include "../../hw/arm/prusa/stm32_chips/stm32f407xx.h"


static void test_io(void)
{
    QTestState* ts = qtest_start("-machine stm32g070xB");
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DBG].base_addr;
	// Is the ID code working?
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 0x0)), ==, 0x10000460);
    // Is it read-only?
    qtest_writel(ts, STM32_RI_ADDRESS(base, 0x0) , 0xffffffff);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 0x0)), ==, 0x10000460);

	// For now just do a simple write test of the other registers:
    qtest_writel(ts, STM32_RI_ADDRESS(base, 1) , 0xBABECAFE);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 1)), ==, 0xBABECAFE);

    qtest_quit(ts);
};

// static void test_f407_io(void)
// {
//     QTestState* ts = qtest_start("-machine stm32f407xE");
// 	uint32_t base = stm32f407xx_cfg.perhipherals[STM32_P_DBG].base_addr;
// 	// Is the ID code working?
//     g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 0x0)), ==, 0x10000413);
//     // Is it read-only?
//     qtest_writel(ts, STM32_RI_ADDRESS(base, 0x0) , 0xffffffff);
// 	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 0x0)), ==, 0x10000413);

// 	// For now just do a simple write test of the other registers:
//     qtest_writel(ts, STM32_RI_ADDRESS(base, 1) , 0xBABECAFE);
//     g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, 1)), ==, 0xBABECAFE);

//     qtest_quit(ts);
// }

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_dbg/test_g070_io", test_io);
    // Not currently implemented in the 407
    // qtest_add_func("/stm32_dbg/test_f407_io", test_f407_io);

    ret = g_test_run();

    return ret;
}
