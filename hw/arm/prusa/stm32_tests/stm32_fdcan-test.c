/*
 * QTest testcase for the STM32 FDCAN peripheral.
 *
 * Copyright 2026 VintagePC <https://github.com/vintagepc>
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

#include "../stm32_chips/stm32c092xx.h"
#include "../stm32_common/stm32_fdcan_index.h"

/*
 * Fixed SRAM layout offsets (byte offsets from SRAMCAN base).
 * Mirrored from stm32_fdcan.c — kept local to avoid a shared header dep.
 */
#define FDCAN_SRAM_RXF0SA     0x00B0U
#define FDCAN_SRAM_TBSA       0x0278U
#define FDCAN_ELEM_WORDS      18U
#define FDCAN_ELEM_BYTES      (FDCAN_ELEM_WORDS * 4U)

/* Byte address of TX buffer element N within SRAMCAN */
#define FDCAN_TX_ELEM(sram_base, n) \
    ((sram_base) + FDCAN_SRAM_TBSA + (n) * FDCAN_ELEM_BYTES)

/* Byte address of RX FIFO 0 element N within SRAMCAN */
#define FDCAN_RX0_ELEM(sram_base, n) \
    ((sram_base) + FDCAN_SRAM_RXF0SA + (n) * FDCAN_ELEM_BYTES)

#define MACHINE "-machine stm32c092xC"

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static void fdcan_enter_config(QTestState *ts, uint32_t base)
{
    /* INIT=1 at reset; set CCE=1 to allow config register writes */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000003U);
    g_assert_cmphex(
        qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x3U);
}

static void fdcan_enable_loopback(QTestState *ts, uint32_t base)
{
    /* INIT=1, CCE=1, TEST=1 so TEST register is writable */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000083U);
    /* LBCK at TEST[4] */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TEST), 0x00000010U);
    /* Leave init: INIT=0, CCE=0, TEST=1 kept so loopback stays active */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000080U);
    g_assert_cmphex(
        qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x0U);
}

static void fdcan_start(QTestState *ts, uint32_t base)
{
    /* Clear INIT — CCE is auto-cleared by hardware when INIT=0 */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000000U);
    g_assert_cmphex(
        qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x0U);
}

/* ------------------------------------------------------------------ */
/* Tests                                                                */
/* ------------------------------------------------------------------ */

static void test_reset_values(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;

    /* CREL / ENDN: hardwired core release and endianness constants */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CREL)), ==, 0x32141218U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ENDN)), ==, 0x87654321U);

    /* CCCR: INIT=1 at reset, all other bits 0 */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)), ==, 0x00000001U);

    /* NBTP / DBTP: nominal and data bit timing reset values */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_NBTP)), ==, 0x06000A03U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_DBTP)), ==, 0x00000A33U);

    /* PSR: initial protocol status (LEC=7, DLEC=7) */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_PSR)), ==, 0x00000707U);

    /* TOCC: timeout period pre-loaded, TOCV: counter initial value */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TOCC)), ==, 0xFFFF0000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TOCV)), ==, 0x0000FFFFU);

    /* XIDAM: extended ID AND mask fully open */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_XIDAM)), ==, 0x1FFFFFFFU);

    /* TXFQS: 3 free TX slots, put/get indices at 0, not full */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)), ==, 0x00000003U);

    /* IR / IE / ILS / ILE: no interrupts pending or enabled */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)),  ==, 0x00000000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IE)),  ==, 0x00000000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ILS)), ==, 0x00000000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ILE)), ==, 0x00000000U);

    /* RXF0S / RXF1S: FIFOs empty at reset */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 0U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF1S)) & 0xFU, ==, 0U);

    qtest_quit(ts);
}

static void test_cccr_init_cce(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;

    /* Reset: INIT=1, CCE=0 */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x1U);

    /* Enter config mode: INIT=1, CCE=1 */
    fdcan_enter_config(ts, base);

    /* Clearing INIT must auto-clear CCE */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x0U);

    /* CCE cannot be set while INIT=0 */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000002U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x2U, ==, 0x0U);

    /* Restore INIT=1; now CCE can be set again */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CCCR), 0x00000003U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CCCR)) & 0x3U, ==, 0x3U);

    qtest_quit(ts);
}

static void test_cccr_entering_config_resets_status(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;

    /* PSR has a non-zero reset value; entering config mode must reset it */
    uint32_t psr_reset = 0x00000707U;
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_PSR)), ==, psr_reset);

    /* Enter then leave config mode once (CCE going 0→1 resets ECR/PSR) */
    fdcan_enter_config(ts, base);
    fdcan_start(ts, base);

    /* Re-enter config — ECR and PSR must be back at reset values */
    fdcan_enter_config(ts, base);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_PSR)), ==, psr_reset);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ECR)), ==, 0x00000000U);

    qtest_quit(ts);
}

static void test_readonly_status_regs(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t before;

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBRP));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBRP), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBRP)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBTO));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBTO), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBTO)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXFQS), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RXF0S), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF1S));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RXF1S), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF1S)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_HPMS));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_HPMS), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_HPMS)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ECR));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ECR), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ECR)), ==, before);

    before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_PSR));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_PSR), 0xFFFFFFFFU);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_PSR)), ==, before);

    qtest_quit(ts);
}

static void test_ir_w1c(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* Write a minimal frame (standard ID=0x1, DLC=0) to TX buffer 0 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, (0x001U << 18)); /* T0 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, 0x00000000U);    /* T1 */

    /* Request transmission; IR.TC (bit 7) must be set afterwards */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)) & (1U << 7));

    /* Writing 1 to IR.TC must clear it (W1C); other bits unaffected */
    uint32_t ir_before = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_IR), 1U << 7);
    uint32_t ir_after = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR));
    g_assert_cmphex(ir_after & (1U << 7), ==, 0U);
    /* Bits other than TC that were set must still be set */
    g_assert_cmphex(ir_after & ~(1U << 7), ==, ir_before & ~(1U << 7));

    /* Writing 0 to IR must not change any bits (W1C: 0 = no effect) */
    uint32_t ir_snap = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR));
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_IR), 0x00000000U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)), ==, ir_snap);

    qtest_quit(ts);
}

static void test_txbar_txbto(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* TXBRP must be clear before any request */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBRP)), ==, 0x0U);

    /* Write TX frame: standard ID=0x123, DLC=4, data=0xDEADBEEF */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0,  (0x123U << 18));  /* T0 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4,  (4U << 16));      /* T1: DLC=4 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 8,  0xDEADBEEFU);

    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);

    /* After synchronous TX: TXBRP cleared, TXBTO bit 0 set, IR.TC set */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBRP)), ==, 0x0U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBTO)), ==, 0x1U);
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)) & (1U << 7));

    /* TXBAR of buffer 2 while 0 and 1 are unused — only bit 2 should set TXBTO */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 2) + 0, 0U);
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 2) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x4U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXBTO)), ==, 0x5U);

    qtest_quit(ts);
}

static void test_txfqs_put_index(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* TFQPI starts at 0, TFFL = 3 (all free) */
    uint32_t txfqs = qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS));
    g_assert_cmphex((txfqs >> 16) & 0x3U, ==, 0U); /* TFQPI=0 */
    g_assert_cmphex(txfqs & 0x7U, ==, 3U);          /* TFFL=3 */
    g_assert_false(txfqs & (1U << 21));              /* not full */

    /* After transmitting buffer 0, TFQPI must advance to 1 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, 0U);
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);
    g_assert_cmphex(
        (qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)) >> 16) & 0x3U, ==, 1U);

    /* After buffer 1, TFQPI = 2 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 1) + 0, 0U);
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 1) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x2U);
    g_assert_cmphex(
        (qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)) >> 16) & 0x3U, ==, 2U);

    /* After buffer 2, TFQPI wraps back to 0 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 2) + 0, 0U);
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 2) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x4U);
    g_assert_cmphex(
        (qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)) >> 16) & 0x3U, ==, 0U);

    /* TFFL is always 3 after synchronous completion; queue never stays full */
    g_assert_cmphex(
        qtest_readl(ts, STM32_RI_ADDRESS(base, RI_TXFQS)) & 0x7U, ==, 3U);

    qtest_quit(ts);
}

static void test_loopback_rx_standard_frame(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* RXF0S fill level must be 0 before TX */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 0U);

    /* Write TX frame: standard ID=0x123, DLC=4, data=0xDEADBEEF */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, (0x123U << 18)); /* T0: std ID */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, (4U << 16));     /* T1: DLC=4 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 8, 0xDEADBEEFU);

    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);

    /* IR.RF0N (bit 0) must be set from loopback receive */
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)) & 0x1U);

    /* RXF0S fill level must now be 1 */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 1U);

    /* Verify received frame in RX FIFO 0, slot 0 */
    uint32_t r0 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 0);
    uint32_t r1 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 4);
    uint32_t d0 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 8);

    g_assert_cmphex((r0 >> 18) & 0x7FFU, ==, 0x123U); /* standard ID */
    g_assert_false(r0 & (1U << 30));                   /* XTD=0 */
    g_assert_false(r0 & (1U << 29));                   /* RTR=0 */
    g_assert_cmphex((r1 >> 16) & 0xFU, ==, 4U);        /* DLC=4 */
    g_assert_cmphex(d0, ==, 0xDEADBEEFU);

    qtest_quit(ts);
}

static void test_loopback_rx_extended_frame(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* Write TX frame: extended ID=0x12345678, DLC=4, data=0xCAFEBABE */
    uint32_t ext_id = 0x12345678U & 0x1FFFFFFFU;
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, ext_id | (1U << 30)); /* T0: XTD=1 */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, (4U << 16));
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 8, 0xCAFEBABEU);

    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);

    uint32_t r0 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 0);
    uint32_t r1 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 4);
    uint32_t d0 = qtest_readl(ts, FDCAN_RX0_ELEM(sram, 0) + 8);

    g_assert_cmphex(r0 & 0x1FFFFFFFU, ==, ext_id); /* extended ID */
    g_assert_true(r0 & (1U << 30));                 /* XTD=1 */
    g_assert_cmphex((r1 >> 16) & 0xFU, ==, 4U);
    g_assert_cmphex(d0, ==, 0xCAFEBABEU);

    qtest_quit(ts);
}

static void test_rxf0a_acknowledge(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* Send two frames to fill two RX slots */
    for (int i = 0; i < 2; i++)
    {
        qtest_writel(ts, FDCAN_TX_ELEM(sram, i) + 0, ((uint32_t)(i + 1) << 18));
        qtest_writel(ts, FDCAN_TX_ELEM(sram, i) + 4, 0U);
        qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 1U << i);
    }
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 2U);

    /* Acknowledge slot 0 (F0AI=0 → F0GI advances to 1, F0FL drops to 1) */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RXF0A), 0x0U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 1U);
    /* Full flag must be clear */
    g_assert_false(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & (1U << 24));

    /* Acknowledge slot 1 — FIFO now empty */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RXF0A), 0x1U);
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 0U);

    qtest_quit(ts);
}

static void test_rxf0s_full_and_lost(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    fdcan_enable_loopback(ts, base);

    /* Fill all 3 RX FIFO 0 slots */
    for (int i = 0; i < 3; i++)
    {
        qtest_writel(ts, FDCAN_TX_ELEM(sram, i) + 0, ((uint32_t)(i + 1) << 18));
        qtest_writel(ts, FDCAN_TX_ELEM(sram, i) + 4, 0U);
        qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 1U << i);
    }

    /* Fill level = 3, full flag set, IR.RF0F (bit 1) set */
    g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & 0xFU, ==, 3U);
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RXF0S)) & (1U << 24));
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)) & (1U << 1));

    /* Overflow: one more TX attempt must set IR.RF0L (bit 2) */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, (0x7FFU << 18));
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);
    g_assert_true(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_IR)) & (1U << 2));

    qtest_quit(ts);
}

static void test_irq_routing(void)
{
    QTestState *ts = qtest_init(MACHINE);
    uint32_t base = stm32c092xx_cfg.perhipherals[STM32_P_CAN1].base_addr;
    uint32_t sram = C092_SRAMCAN_ADDR;

    qtest_irq_intercept_out_named(ts, "/machine/soc/CAN1", "sysbus-irq");

    /* Put in normal (non-loopback) mode so only TC fires (no RF0N) */
    fdcan_enter_config(ts, base);
    fdcan_start(ts, base);

    /* Enable only TC interrupt, route all groups to line 0, enable line 0 */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_IE),  1U << 7); /* TCE */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ILS), 0x00U);   /* all → line 0 */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ILE), 0x01U);   /* EINT0 */

    g_assert_false(qtest_get_irq(ts, 0));
    g_assert_false(qtest_get_irq(ts, 1));

    /* Trigger TX (to null bus — frame is dropped, but IR.TC still fires) */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 0, (0x1U << 18));
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 0) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x1U);

    /* IR.TC → ILS.SMSG(0) → line 0: line 0 must be high, line 1 low */
    g_assert_true(qtest_get_irq(ts, 0));
    g_assert_false(qtest_get_irq(ts, 1));

    /* Clear IR.TC — line 0 must drop */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_IR), 1U << 7);
    g_assert_false(qtest_get_irq(ts, 0));

    /* Re-route SMSG group (ILS[2]) to line 1; enable both lines */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ILS), 0x04U); /* SMSG → line 1 */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ILE), 0x03U); /* EINT0 + EINT1 */

    /* Trigger another TX */
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 1) + 0, (0x2U << 18));
    qtest_writel(ts, FDCAN_TX_ELEM(sram, 1) + 4, 0U);
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TXBAR), 0x2U);

    /* Now only line 1 fires */
    g_assert_false(qtest_get_irq(ts, 0));
    g_assert_true(qtest_get_irq(ts, 1));

    /* Disabling line 1 via ILE must suppress the IRQ */
    qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ILE), 0x01U); /* only EINT0 */
    g_assert_false(qtest_get_irq(ts, 1));

    qtest_quit(ts);
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_fdcan/reset_values",               test_reset_values);
    qtest_add_func("/stm32_fdcan/cccr_init_cce",              test_cccr_init_cce);
    qtest_add_func("/stm32_fdcan/cccr_config_resets_status",  test_cccr_entering_config_resets_status);
    qtest_add_func("/stm32_fdcan/readonly_status_regs",       test_readonly_status_regs);
    qtest_add_func("/stm32_fdcan/ir_w1c",                     test_ir_w1c);
    qtest_add_func("/stm32_fdcan/txbar_txbto",                test_txbar_txbto);
    qtest_add_func("/stm32_fdcan/txfqs_put_index",            test_txfqs_put_index);
    qtest_add_func("/stm32_fdcan/loopback_rx_standard_frame", test_loopback_rx_standard_frame);
    qtest_add_func("/stm32_fdcan/loopback_rx_extended_frame", test_loopback_rx_extended_frame);
    qtest_add_func("/stm32_fdcan/rxf0a_acknowledge",          test_rxf0a_acknowledge);
    qtest_add_func("/stm32_fdcan/rxf0s_full_and_lost",        test_rxf0s_full_and_lost);
    qtest_add_func("/stm32_fdcan/irq_routing",                test_irq_routing);

    ret = g_test_run();

    return ret;
}
