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

#include "../../hw/arm/prusa/stm32_chips/stm32f030xx.h"
#include "../../hw/arm/prusa/stm32_common/stm32_iwdg_regdata.h"
#include "../../hw/arm/prusa/stm32_common/stm32_shared.h"

typedef struct test_info_t
{
	hwaddr reg;
	uint8_t periphs[32];
	const char* machine_args;

} test_info_t;

static void test_resets(gconstpointer data)
{
	test_info_t* d = (test_info_t*)data;
	QTestState *ts = qtest_init(d->machine_args);
	qtest_irq_intercept_out_named(ts, "/machine/soc/RCC", "reset");
	for (int i=0; i<32; i++)
	{
		if (d->periphs[i] > 0)
		{
			printf("# Testing %s reset\n", _PERIPHNAMES[d->periphs[i]]);
			g_assert_false(qtest_get_irq_pulsed(ts, d->periphs[i]));
			qtest_writel(ts, d->reg, 1U << i);
			// Check that ONLY the specified reset was pulsed.
			for (int j=STM32_P_RCC; j<STM32_P_COUNT; j++)
			{
				switch (d->periphs[i])
				{
					case STM32_P_ADC_ALL:
						g_assert_cmpint(qtest_get_irq_pulsed(ts, STM32_P_ADC1), ==, true);
						g_assert_cmpint(qtest_get_irq_pulsed(ts, STM32_P_ADC2), ==, true);
						g_assert_cmpint(qtest_get_irq_pulsed(ts, STM32_P_ADC3), ==, true);
						break;
					default:
						g_assert_cmpint(qtest_get_irq_pulsed(ts, j), ==, j == d->periphs[i]);
						break;
				}
			}
			qtest_irq_clear_pulses(ts);
		}
	}
	qtest_quit(ts);
}

static const test_info_t f030_apb1 = {
	.reg = 0x40021010,
	.machine_args = "-machine stm32f030x4",
	.periphs = {
		0, STM32_P_TIM3, 0, 0, STM32_P_TIM6, STM32_P_TIM7, 0, 0,
		STM32_P_TIM14, 0, 0, STM32_P_WWDG, 0, 0, STM32_P_SPI2, 0,
		0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, STM32_P_UART5, STM32_P_I2C1, STM32_P_I2C2, STM32_P_USBHS,
		0, 0, 0, STM32_P_CRYP, STM32_P_PWR, 0, 0, 0
	}
};

static const test_info_t f030_apb2 = {
	.reg = 0x4002100C,
	.machine_args = "-machine stm32f030x4",
	.periphs = {
		STM32_P_SYSCFG, 0, 0, 0, 0, STM32_P_UART6, 0, 0,
		0, STM32_P_ADC1, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_UART1, 0,
		STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t f030_ahb1 = {
	.reg = 0x40021028,
	.machine_args = "-machine stm32f030x4",
	.periphs = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, 0, STM32_P_GPIOF, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t g070_ahb1 = {
	.reg = 0x40021028,
	.machine_args = "-machine stm32g070xB",
	.periphs = {
		STM32_P_DMA1, STM32_P_DMA2, 0, 0, 0, 0, 0, 0,
		STM32_P_FSMC, 0, 0, 0, STM32_P_CRC, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t g070_apb1 = {
	.reg = 0x4002102C,
	.machine_args = "-machine stm32g070xB",
	.periphs = {
		0, STM32_P_TIM3, STM32_P_TIM4, 0, STM32_P_TIM6, STM32_P_TIM7, 0, 0,
		STM32_P_UART5, STM32_P_UART6, 0, 0, 0, STM32_P_USBHS, STM32_P_SPI2, STM32_P_SPI3,
		0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, 0, STM32_P_I2C1, STM32_P_I2C2, STM32_P_I2C3,
		0, 0, 0, 0, STM32_P_PWR, 0, 0, 0
	}
};

static const test_info_t g070_apb2 = {
	.reg = 0x40021030,
	.machine_args = "-machine stm32g070xB",
	.periphs = {
		STM32_P_SYSCFG, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, STM32_P_TIM1, STM32_P_SPI1, 0, STM32_P_UART1, STM32_P_TIM14,
		STM32_P_TIM15, STM32_P_TIM16, STM32_P_TIM17, 0, STM32_P_ADC1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t g070_gpio = {
	.reg = 0x40021024,
	.machine_args = "-machine stm32g070xB",
	.periphs = {
		STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, STM32_P_GPIOE, STM32_P_GPIOF, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};


static const test_info_t f4xx_ahb1 = {
	.reg = 0x40023810,
	.machine_args = "-machine stm32f427xE",
	.periphs = {
		STM32_P_GPIOA, STM32_P_GPIOB, STM32_P_GPIOC, STM32_P_GPIOD, STM32_P_GPIOE, STM32_P_GPIOF, STM32_P_GPIOG, STM32_P_GPIOH,
		STM32_P_GPIOI, STM32_P_GPIOJ, STM32_P_GPIOK, 0, STM32_P_CRC, 0, 0, 0,
		0, 0, 0, 0, 0, STM32_P_DMA1, STM32_P_DMA2, 0,
		0, 0, 0/*eth*/, 0, 0, STM32_P_USBHS, 0, 0
	}
};

static const test_info_t f4xx_ahb2 = {
	.reg = 0x40023814,
	.machine_args = "-machine stm32f427xE",
	.periphs = {
		STM32_P_DCMI, 0, 0, 0, STM32_P_CRYP, STM32_P_HASH, STM32_P_RNG, STM32_P_USBFS,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t f4xx_ahb3 = {
	.reg = 0x40023818,
	.machine_args = "-machine stm32f427xE",
	.periphs = {
		STM32_P_FSMC, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

static const test_info_t f4xx_apb1 = {
	.reg = 0x40023820,
	.machine_args = "-machine stm32f427xE",
	.periphs = {
		STM32_P_TIM2, STM32_P_TIM3, STM32_P_TIM4, STM32_P_TIM5, STM32_P_TIM6, STM32_P_TIM7, STM32_P_TIM12, STM32_P_TIM13,
		STM32_P_TIM14, 0, 0, STM32_P_WWDG, 0, 0, STM32_P_SPI2, STM32_P_SPI3,
		0, STM32_P_UART2, STM32_P_UART3, STM32_P_UART4, STM32_P_UART5, STM32_P_I2C1, STM32_P_I2C2, STM32_P_I2C3,
		0, STM32_P_CAN1, STM32_P_CAN2, 0, STM32_P_PWR, STM32_P_DAC, STM32_P_UART7, STM32_P_UART8
	}
};

static const test_info_t f4xx_apb2 = {
	.reg = 0x40023824,
	.machine_args = "-machine stm32f427xE",
	.periphs = {
		STM32_P_TIM1, STM32_P_TIM8, 0, 0, STM32_P_UART1, STM32_P_UART6, 0, 0,
		STM32_P_ADC_ALL, 0, 0, STM32_P_SDIO, STM32_P_SPI1, STM32_P_SPI4, STM32_P_SYSCFG, 0,
		STM32_P_TIM9, STM32_P_TIM10, STM32_P_TIM11, 0, STM32_P_SPI5, STM32_P_SPI6, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	}
};

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_data_func("/stm32_rcc/f030_apb1_resets", &f030_apb1, test_resets);
    qtest_add_data_func("/stm32_rcc/f030_apb2_resets", &f030_apb2, test_resets);
    qtest_add_data_func("/stm32_rcc/f030_ahb1_resets", &f030_ahb1, test_resets);

	qtest_add_data_func("/stm32_rcc/g070_apb1_resets", &g070_apb1, test_resets);
    qtest_add_data_func("/stm32_rcc/g070_apb2_resets", &g070_apb2, test_resets);
    qtest_add_data_func("/stm32_rcc/g070_ahb1_resets", &g070_ahb1, test_resets);
    qtest_add_data_func("/stm32_rcc/g070_gpio_resets", &g070_gpio, test_resets);

	// Note we only run the 427, it has all the current peripherals mapped. It's up to the
	// "lesser" variants to mask out bits appropriately and that's not currently checked.

	qtest_add_data_func("/stm32_rcc/f4xx_apb1_resets", &f4xx_apb1, test_resets);
    qtest_add_data_func("/stm32_rcc/f4xx_apb2_resets", &f4xx_apb2, test_resets);
    qtest_add_data_func("/stm32_rcc/f4xx_ahb1_resets", &f4xx_ahb1, test_resets);
    qtest_add_data_func("/stm32_rcc/f4xx_ahb2_resets", &f4xx_ahb2, test_resets);
    qtest_add_data_func("/stm32_rcc/f4xx_ahb3_resets", &f4xx_ahb3, test_resets);

    ret = g_test_run();

    return ret;
}
