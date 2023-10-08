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

static void test_blank(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_OTP].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");

	g_assert_cmphex(qtest_readl(ts, base), ==, 0xFFFFFFFF);
	g_assert_cmphex(qtest_readw(ts, base), ==, 0xFFFF);
	g_assert_cmphex(qtest_readb(ts, base), ==, 0xFF);

	// Todo - add write-once tests here once that is implemented.
	qtest_quit(ts);

}

static void test_filebacked(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_OTP].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB -drive file=stm32_otp_test_data,id=stm32-otp,format=raw");

	g_assert_cmphex(qtest_readl(ts, base+10), ==, 0x00030203);
	g_assert_cmphex(qtest_readw(ts, base+10), ==, 0x0203);
	g_assert_cmphex(qtest_readw(ts, base+11), ==, 0x0302);
	g_assert_cmphex(qtest_readw(ts, base+12), ==, 0x0003);
	g_assert_cmphex(qtest_readb(ts, base+10), ==, 0x03);
	g_assert_cmphex(qtest_readb(ts, base+11), ==, 0x02);
	g_assert_cmphex(qtest_readb(ts, base+12), ==, 0x03);
	g_assert_cmphex(qtest_readb(ts, base+13), ==, 0x00);

	// Verify it cannot be written:
	qtest_writel(ts, base +10, 0xFFFFFFFF);
	g_assert_cmphex(qtest_readl(ts, base+10), ==, 0x00030203);
	qtest_writew(ts, base +10, 0xFFFF);
	qtest_writew(ts, base +11, 0xFFFF);
	qtest_writew(ts, base +12, 0xFFFF);
	g_assert_cmphex(qtest_readl(ts, base+10), ==, 0x00030203);
	qtest_writeb(ts, base +10, 0xFF);
	qtest_writeb(ts, base +11, 0xFF);
	qtest_writeb(ts, base +12, 0xFF);
	qtest_writeb(ts, base +13, 0xFF);
	g_assert_cmphex(qtest_readl(ts, base+10), ==, 0x00030203);
	qtest_quit(ts);

}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

	uint32_t data[512];
	for (int i=0; i<512; i++)
	{
		data[i] = i | (i+ 513) <<16;
	}

	FILE* fp = fopen("./stm32_otp_test_data","wb");
	fwrite(&data, sizeof(data), 1, fp);
	fclose(fp);

    qtest_add_func("/stm32_otp/uninitialized", test_blank);
    qtest_add_func("/stm32_otp/test_file_init", test_filebacked);

    ret = g_test_run();

    return ret;
}
