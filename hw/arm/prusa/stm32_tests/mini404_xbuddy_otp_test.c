/*
 * QTest testcase for the OTP_v4 block the xBuddy machines burn into the STM32
 * OTP area: serial number (datamatrix) and MAC address, including the
 * serial-suffix and mac machine properties.
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
#include "libqtest.h"

#define OTP_BASE        0x1FFF7800U
#define OTP_BOM_ID      (OTP_BASE + 3U)
#define OTP_DATAMATRIX  (OTP_BASE + 8U)
#define OTP_MAC         (OTP_BASE + 32U)
#define DATAMATRIX_LEN  22

static void read_bytes(QTestState *ts, uint32_t addr, uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = qtest_readb(ts, addr + i);
	}
}

static void check_identity(const char *machine, const char *serial, const uint8_t mac[6])
{
	QTestState *ts = qtest_initf("-machine %s", machine);
	char dm[DATAMATRIX_LEN + 1] = {};
	uint8_t otp_mac[6];

	read_bytes(ts, OTP_DATAMATRIX, (uint8_t *)dm, DATAMATRIX_LEN);
	read_bytes(ts, OTP_MAC, otp_mac, sizeof(otp_mac));

	g_assert_cmpuint(qtest_readb(ts, OTP_BOM_ID), ==, 38);
	g_assert_cmpstr(dm, ==, serial);
	g_assert_cmpmem(otp_mac, sizeof(otp_mac), mac, 6);

	qtest_quit(ts);
}

// Without properties: the built-in serial, and a nonzero MAC derived from its
// last four digits (9999 = 0x270F). All six MAC bytes must reach the guest.
static void test_default(void)
{
	const uint8_t mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x27, 0x0F };
	check_identity("prusa-mk4-027c", "4558-27000019005259999", mac);
}

static void test_serial_suffix(void)
{
	const uint8_t mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x10, 0x92 };
	check_identity("prusa-mk4-027c,serial-suffix=4242", "4558-27000019005254242", mac);
}

static void test_serial_suffix_padded(void)
{
	const uint8_t mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x07 };
	check_identity("prusa-mk4-027c,serial-suffix=7", "4558-27000019005250007", mac);
}

static void test_explicit_mac(void)
{
	const uint8_t mac[6] = { 0x0a, 0x1b, 0x2c, 0x3d, 0x4e, 0x5f };
	check_identity("prusa-mk4-027c,mac=0a:1b:2c:3d:4e:5f", "4558-27000019005259999", mac);
}

// The datamatrix revision digits differ on the 034 board; the suffix still
// lands on the last four digits.
static void test_rev34(void)
{
	const uint8_t mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x04, 0xD2 };
	check_identity("prusa-mk4-034,serial-suffix=1234", "4558-34000019005251234", mac);
}

int main(int argc, char **argv)
{
	g_test_init(&argc, &argv, NULL);
	g_test_set_nonfatal_assertions();

	qtest_add_func("/mini404/xbuddy_otp/default", test_default);
	qtest_add_func("/mini404/xbuddy_otp/serial_suffix", test_serial_suffix);
	qtest_add_func("/mini404/xbuddy_otp/serial_suffix_padded", test_serial_suffix_padded);
	qtest_add_func("/mini404/xbuddy_otp/explicit_mac", test_explicit_mac);
	qtest_add_func("/mini404/xbuddy_otp/rev34", test_rev34);

	return g_test_run();
}
