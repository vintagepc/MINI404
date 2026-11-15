/*
 * QTest testcase for the STM32F2xx/F4xx RTC calendar registers.
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

#define RCC_BDCR  0x40023870U
#define BDCR_LSEON  (1U << 0)
#define BDCR_RTCSEL_LSE (1U << 8)
#define BDCR_RTCEN  (1U << 15)

#define RTC_BASE  0x40002800U
#define RTC_TR    (RTC_BASE + 0x00U)
#define RTC_DR    (RTC_BASE + 0x04U)
#define RTC_PRER  (RTC_BASE + 0x10U)
#define RTC_WPR   (RTC_BASE + 0x24U)

#define DR_YEAR(dr)  (((dr) >> 16) & 0xFFU)
#define DR_WDU(dr)   (((dr) >> 13) & 0x7U)
#define DR_MONTH(dr) (((dr) >> 8) & 0x1FU)
#define DR_DAY(dr)   ((dr) & 0x3FU)

// Set the calendar, then let it run so the read comes back through the
// model's encoder rather than echoing the raw register write.
static uint32_t rtc_set_and_read_date(uint32_t tr, uint32_t dr)
{
	// Unclocked, the RTC drops register writes.
	writel(RCC_BDCR, BDCR_LSEON | BDCR_RTCSEL_LSE | BDCR_RTCEN);
	// Lock first, as HAL does: the unlock sequence must start from locked.
	writel(RTC_WPR, 0xFF);
	writel(RTC_WPR, 0xCA);
	writel(RTC_WPR, 0x53);
	// Prescalers at 0: one calendar second per LSE cycle (~30 us of host
	// time), so a short sleep advances it by minutes at most. Set at noon so
	// the date can't roll over.
	writel(RTC_PRER, 0);
	writel(RTC_TR, tr);
	writel(RTC_DR, dr);
	writel(RTC_WPR, 0xFF);
	g_usleep(20 * 1000);
	return readl(RTC_DR);
}

// Plain date: 2026-11-13, a Friday.
static void test_date_roundtrip(void)
{
	uint32_t dr = rtc_set_and_read_date(0x00120000, 0x00261113);
	g_assert_cmphex(DR_YEAR(dr), ==, 0x26);
	g_assert_cmphex(DR_MONTH(dr), ==, 0x11);
	g_assert_cmphex(DR_DAY(dr), ==, 0x13);
	g_assert_cmpuint(DR_WDU(dr), ==, 5); // Monday = 1 .. Sunday = 7
}

// YT is a 4-bit field, so years up to 159 must read back as written. Buddy
// firmware stores tm_year (126 for 2026) and reads it back as 0xC6.
static void test_year_above_99(void)
{
	uint32_t dr = rtc_set_and_read_date(0x00120000, 0x00C61013);
	g_assert_cmphex(DR_YEAR(dr), ==, 0xC6);
	g_assert_cmphex(DR_MONTH(dr), ==, 0x10);
	g_assert_cmphex(DR_DAY(dr), ==, 0x13);
}

int main(int argc, char **argv)
{
	int ret;

	g_test_init(&argc, &argv, NULL);
	g_test_set_nonfatal_assertions();

	qtest_add_func("/stm32/rtc/date_roundtrip", test_date_roundtrip);
	qtest_add_func("/stm32/rtc/year_above_99", test_year_above_99);

	qtest_start("-machine stm32f427xI");
	ret = g_test_run();
	qtest_end();

	return ret;
}
