/*
 * QTest testcase for F4 peripheral behaviour the xBuddy bootloader relies on:
 * CRC reset value, reset-cause flags, externally held GPIO inputs, polled ADC
 * reads and bank-2 flash sector erase.
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
#include "qemu/units.h"

#include "../stm32_chips/stm32f407xx.h" // register helper macros only
#include "../stm32f407/stm32f4xx_flashint_regdata.h"

#define RCC_BASE     0x40023800U
#define RCC_AHB1ENR  (RCC_BASE + 0x30U)
#define RCC_APB2ENR  (RCC_BASE + 0x44U)
#define RCC_CSR      (RCC_BASE + 0x74U)

#define CSR_PADRSTF  BIT(26)
#define CSR_PORRSTF  BIT(27)
#define CSR_SFTRSTF  BIT(28)
#define CSR_RMVF     BIT(24)

#define CRC_BASE     0x40023000U
#define CRC_DR       (CRC_BASE + 0x00U)
#define CRC_CR       (CRC_BASE + 0x08U)

#define GPIOA_BASE   0x40020000U
#define GPIO_MODER   0x00U
#define GPIO_IDR     0x10U
#define GPIO_BSRR    0x18U

#define ADC1_BASE    0x40012000U
#define ADC_SR       0x00U
#define ADC_CR2      0x08U
#define ADC_SQR3     0x34U
#define ADC_DR       0x4CU
#define ADC_SR_EOC   BIT(1)
#define ADC_CR2_ADON BIT(0)
#define ADC_CR2_SWSTART BIT(30)

#define KEY1 0x45670123UL
#define KEY2 0xCDEF89ABUL

// F2/F4 have no INIT register: CR.RESET must reload 0xFFFFFFFF.
static void test_crc_reset(void)
{
	writel(RCC_AHB1ENR, readl(RCC_AHB1ENR) | BIT(12));
	writel(CRC_CR, 1);
	g_assert_cmphex(readl(CRC_DR), ==, 0xFFFFFFFF);
	writel(CRC_DR, 0xDEADBEEF);
	g_assert_cmphex(readl(CRC_DR), ==, 0x81DA1A18);
	writel(CRC_CR, 1);
	g_assert_cmphex(readl(CRC_DR), ==, 0xFFFFFFFF);
}

// The first reset is power-on; a later one is a software reset. Flags are
// sticky until RMVF.
static void test_reset_cause(void)
{
	uint32_t csr = readl(RCC_CSR);
	g_assert_true(csr & CSR_PORRSTF);
	g_assert_false(csr & CSR_SFTRSTF);

	qtest_system_reset(global_qtest);
	csr = readl(RCC_CSR);
	g_assert_true(csr & CSR_SFTRSTF);
	g_assert_true(csr & CSR_PADRSTF);
	g_assert_true(csr & CSR_PORRSTF); // still there: sticky

	writel(RCC_CSR, CSR_RMVF);
	g_assert_cmphex(readl(RCC_CSR) & 0xFF000000U, ==, 0);

	qtest_system_reset(global_qtest);
	csr = readl(RCC_CSR);
	g_assert_true(csr & CSR_SFTRSTF);
	g_assert_false(csr & CSR_PORRSTF);
}

// idr-mask=0x1 (set on the command line) holds PA0 low from outside: the
// chip's own output latch must not pull it up.
static void test_gpio_held_low(void)
{
	g_assert_cmphex(readl(GPIOA_BASE + GPIO_IDR) & 1, ==, 0);
	writel(GPIOA_BASE + GPIO_BSRR, 1);
	g_assert_cmphex(readl(GPIOA_BASE + GPIO_IDR) & 1, ==, 0);
	// An unmasked pin still follows the latch.
	writel(GPIOA_BASE + GPIO_BSRR, 2);
	g_assert_cmphex(readl(GPIOA_BASE + GPIO_IDR) & 2, ==, 2);
}

// HAL_ADC_PollForConversion clears EOC before HAL_ADC_GetValue reads DR; the
// result must still be there.
static void test_adc_dr_after_eoc(void)
{
	writel(RCC_APB2ENR, readl(RCC_APB2ENR) | BIT(8));
	qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 4, 100);
	writel(ADC1_BASE + ADC_SQR3, 4);
	writel(ADC1_BASE + ADC_CR2, ADC_CR2_ADON);
	writel(ADC1_BASE + ADC_CR2, ADC_CR2_ADON | ADC_CR2_SWSTART);
	for (int i = 0; i < 1000 && !(readl(ADC1_BASE + ADC_SR) & ADC_SR_EOC); i++) {
		clock_step(1000);
	}
	g_assert_true(readl(ADC1_BASE + ADC_SR) & ADC_SR_EOC);
	writel(ADC1_BASE + ADC_SR, readl(ADC1_BASE + ADC_SR) & ~ADC_SR_EOC); // as __HAL_ADC_CLEAR_FLAG
	g_assert_false(readl(ADC1_BASE + ADC_SR) & ADC_SR_EOC);
	g_assert_cmphex(readl(ADC1_BASE + ADC_DR), ==, 100);
}

// Bank-2 sectors are SNB 16..23 (the HAL adds 4 above sector 11).
static void test_flash_bank2_erase(void)
{
	const uint32_t base = 0x40023C00U; // FLASH interface
	const uint32_t flash = 0x08000000U;
	REGDEF_NAME(flashif, cr) r;
	REGDEF_NAME(flashif, sr) s;

	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY1);
	writel(STM32_RI_ADDRESS(base, RI_KEYR), KEY2);

	// Mark the first word of each bank-2 sector and the last word of bank 1.
	const uint32_t probes[] = { 1U*MiB - 4U, 1U*MiB, 1U*MiB + 16U*KiB, 1U*MiB + 64U*KiB, 1U*MiB + 128U*KiB };
	for (int i = 0; i < ARRAY_SIZE(probes); i++) {
		writel(flash + probes[i], 0x12345678);
	}

	// SNB 16 = bank 2 sector 0 (1 MiB .. 1 MiB + 16 KiB).
	r.raw = 0;
	r.SER = 1;
	r.SNB = 16;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	r.STRT = 1;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	g_assert_cmphex(readl(flash + 1U*MiB - 4U), ==, 0x12345678);
	g_assert_cmphex(readl(flash + 1U*MiB), ==, 0xFFFFFFFF);
	g_assert_cmphex(readl(flash + 1U*MiB + 16U*KiB), ==, 0x12345678);

	// SNB 21 = bank 2 sector 5 (1 MiB + 128 KiB ..).
	r.SNB = 21;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	g_assert_cmphex(readl(flash + 1U*MiB + 128U*KiB), ==, 0xFFFFFFFF);
	g_assert_cmphex(readl(flash + 1U*MiB + 64U*KiB), ==, 0x12345678);

	// SNB 12..15 do not exist: nothing erased, PGSERR set, then cleared.
	r.SNB = 13;
	writel(STM32_RI_ADDRESS(base, RI_CR), r.raw);
	s.raw = readl(STM32_RI_ADDRESS(base, RI_SR));
	g_assert_true(s.PGSERR);
	g_assert_cmphex(readl(flash + 1U*MiB + 16U*KiB), ==, 0x12345678);
	writel(STM32_RI_ADDRESS(base, RI_SR), s.raw);
	s.raw = readl(STM32_RI_ADDRESS(base, RI_SR));
	g_assert_false(s.PGSERR);
}

int main(int argc, char **argv)
{
	int ret;

	g_test_init(&argc, &argv, NULL);
	g_test_set_nonfatal_assertions();

	// Order matters: the reset-cause test needs the power-on state first.
	qtest_add_func("/stm32f427_boot/reset_cause", test_reset_cause);
	qtest_add_func("/stm32f427_boot/crc_reset", test_crc_reset);
	qtest_add_func("/stm32f427_boot/gpio_held_low", test_gpio_held_low);
	qtest_add_func("/stm32f427_boot/adc_dr_after_eoc", test_adc_dr_after_eoc);
	qtest_add_func("/stm32f427_boot/flash_bank2_erase", test_flash_bank2_erase);

	qtest_start("-machine stm32f427xI -global stm32f4xx-gpio.idr-mask=0x1");

	ret = g_test_run();
	qtest_end();

	return ret;
}
