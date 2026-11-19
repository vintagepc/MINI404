/*
 * QTest for the STM32F4 Ethernet RX path: receive flow control (the net-layer
 * .can_receive callback) and the RX-buffer size clamp.
 *
 * The attack-surface work this guards (lwIP OOSEQ/fragment handling) relies on
 * the model NOT silently dropping frames when the 4-descriptor RX ring is
 * momentarily full: a dropped fragment would make a vulnerable build look safe.
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
#include "qemu/iov.h"
#include "qemu/bswap.h"
#include <sys/socket.h>

#define ETH_BASE          0x40028000U
#define DMA_RCV_POLL_DMD  (ETH_BASE + 0x1008U)
#define DMA_RCV_BASE      (ETH_BASE + 0x100CU)
#define DMA_STATUS        (ETH_BASE + 0x1014U)
#define DMA_CONTROL       (ETH_BASE + 0x1018U)
#define DMA_CONTROL_SR    0x00000002U
#define DMA_STATUS_RU     0x00000080U   /* RX buffer unavailable (RBUS) */

#define RDES0_OWN         0x80000000U
#define RDES0_ES          0x00008000U   /* error summary */
#define RDES0_FL_SHIFT    16            /* frame length, bits [29:16] */

#define NUM_DESC          4
#define DESC_SIZE         32            /* sizeof(struct desc), QEMU_PACKED */
#define DESC_BASE         0x20010000U   /* SRAM */
#define BUF_BASE          0x20011000U
#define BUF_STRIDE        2048U
#define BUF1_CAP          1536U         /* RBS1 for each descriptor */

/* struct desc is little-endian in guest memory:
   +0 ctl_stat(u32) +4 buffer1_size(u16) +6 buffer2_size(u16)
   +8 buffer1_addr(u32) +12 buffer2_nextdescaddr(u32) +16 ext_stat +20 res[3] */
static void write_desc(QTestState *qts, uint32_t addr, uint32_t ctl,
                       uint16_t b1size, uint32_t b1addr, uint32_t next)
{
    uint8_t d[DESC_SIZE] = {0};
    stl_le_p(&d[0], ctl);
    stw_le_p(&d[4], b1size);
    stl_le_p(&d[8], b1addr);
    stl_le_p(&d[12], next);
    qtest_memwrite(qts, addr, d, sizeof(d));
}

static uint32_t desc_ctl(QTestState *qts, int i)
{
    uint8_t d[4];
    qtest_memread(qts, DESC_BASE + i * DESC_SIZE, d, sizeof(d));
    return ldl_le_p(d);
}

/* Build the 4-descriptor ring, all owned by the DMA, chained in a loop. */
static void setup_ring(QTestState *qts)
{
    for (int i = 0; i < NUM_DESC; i++) {
        uint32_t next = DESC_BASE + ((i + 1) % NUM_DESC) * DESC_SIZE;
        write_desc(qts, DESC_BASE + i * DESC_SIZE, RDES0_OWN, BUF1_CAP,
                   BUF_BASE + i * BUF_STRIDE, next);
    }
    qtest_writel(qts, DMA_RCV_BASE, DESC_BASE);
    qtest_writel(qts, DMA_CONTROL, qtest_readl(qts, DMA_CONTROL) | DMA_CONTROL_SR);
}

/* One Ethernet frame carrying a tag byte at payload offset 0 (frame byte 14). */
static void make_frame(uint8_t *f, size_t len, uint8_t tag)
{
    static const uint8_t dst[6] = {0x02, 0, 0, 0, 0, 0x99};
    static const uint8_t src[6] = {0x02, 0, 0, 0, 0, 0x01};
    memset(f, 0, len);
    memcpy(f, dst, 6);
    memcpy(f + 6, src, 6);
    f[12] = 0x08; f[13] = 0x00;   /* ethertype IPv4 */
    f[14] = tag;
}

/* Inject a frame into the socket backend (4-byte big-endian length prefix). */
static void inject(int fd, const uint8_t *frame, size_t len)
{
    uint32_t be = htonl((uint32_t)len);
    const struct iovec iov[] = {
        { .iov_base = &be, .iov_len = sizeof(be) },
        { .iov_base = (void *)frame, .iov_len = len },
    };
    ssize_t r = iov_send(fd, iov, 2, 0, sizeof(be) + len);
    g_assert_cmpint(r, ==, (ssize_t)(sizeof(be) + len));
}

/* Pump the main loop until descriptor i is handed back to the CPU. Returns
   false on timeout (reported by the caller, so later tests still run). */
static bool wait_consumed(QTestState *qts, int i)
{
    for (int spin = 0; spin < 1000; spin++) {
        if ((desc_ctl(qts, i) & RDES0_OWN) == 0) {
            return true;
        }
        qtest_clock_step(qts, 100000);
    }
    return false;
}

/* Let the main loop run so the socket backend is read and delivery happens. */
static void pump(QTestState *qts)
{
    for (int i = 0; i < 20; i++) {
        qtest_clock_step(qts, 100000);
    }
}

static uint32_t dma_status(QTestState *qts)
{
    return qtest_readl(qts, DMA_STATUS);
}

static uint8_t buf_tag(QTestState *qts, int i)
{
    uint8_t t;
    qtest_memread(qts, BUF_BASE + i * BUF_STRIDE + 14, &t, 1);
    return t;
}

static int start_qemu(QTestState **qts)
{
    int sv[2];
    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, sv), ==, 0);
    *qts = qtest_initf("-machine prusa-mk4-027c "
                       "-netdev socket,id=mini-eth,fd=%d", sv[1]);
    return sv[0];
}

/* A single frame is received into the first descriptor. */
static void test_rx_basic(void)
{
    QTestState *qts;
    int fd = start_qemu(&qts);
    setup_ring(qts);

    uint8_t f[64];
    make_frame(f, sizeof(f), 0x11);
    inject(fd, f, sizeof(f));
    g_assert_true(wait_consumed(qts, 0));

    g_assert_cmphex(desc_ctl(qts, 0) & RDES0_OWN, ==, 0);
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x11);
    close(fd);
    qtest_quit(qts);
}

/*
 * Fill the ring, then exercise the full-ring path. Two things must hold:
 *  - a frame that arrives with the ring full is HELD, not dropped, and the
 *    model raises RX-buffer-unavailable (RU/RBUS) so the firmware knows to
 *    free a descriptor and poke the poll-demand (bug: without RU the firmware
 *    never resumes);
 *  - a poll-demand/flush while the ring is STILL full must re-queue the held
 *    frame, not drop it (bug: returning the length here consumes-and-drops it).
 * Only after a descriptor is freed and poll-demand is poked does it land.
 */
static void test_rx_backpressure(void)
{
    QTestState *qts;
    int fd = start_qemu(&qts);
    setup_ring(qts);

    uint8_t f[64];
    for (int i = 0; i < NUM_DESC; i++) {
        make_frame(f, sizeof(f), 0x20 + i);
        inject(fd, f, sizeof(f));
        g_assert_true(wait_consumed(qts, i));
        g_assert_cmpuint(buf_tag(qts, i), ==, 0x20 + i);
    }

    /* Ring full (all four owned by CPU). Send a fifth frame. */
    make_frame(f, sizeof(f), 0x55);
    inject(fd, f, sizeof(f));
    pump(qts);
    g_assert_cmphex(dma_status(qts) & DMA_STATUS_RU, ==, DMA_STATUS_RU); /* RBUS raised */
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x20);   /* held, not delivered */

    /* Poll-demand while the ring is still full: the held frame must survive. */
    qtest_writel(qts, DMA_RCV_POLL_DMD, 1);
    pump(qts);
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x20);   /* still held, not dropped */

    /* Now free descriptor 0, clear RBUS and poke poll-demand exactly as the
       firmware's RX handler does (ethernetif.c clears RBUS then writes DMARPDR).
       The held frame lands. */
    write_desc(qts, DESC_BASE, RDES0_OWN, BUF1_CAP, BUF_BASE,
               DESC_BASE + DESC_SIZE);
    qtest_writel(qts, DMA_STATUS, DMA_STATUS_RU);   /* write-1-to-clear RBUS */
    qtest_writel(qts, DMA_RCV_POLL_DMD, 1);
    g_assert_true(wait_consumed(qts, 0));
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x55);
    close(fd);
    qtest_quit(qts);
}

/*
 * A frame larger than the descriptor's RX buffer must not be written past it.
 * The model drops it silently, leaving the descriptor owned by the DMA and the
 * buffer memory untouched, and the ring keeps working for the next frame.
 */
static void test_rx_oversize(void)
{
    QTestState *qts;
    int fd = start_qemu(&qts);
    setup_ring(qts);

    /* Canary: poison buffer 0 and the stride after it (into buffer 1's start). */
    uint8_t poison[BUF_STRIDE + 64];
    memset(poison, 0xA5, sizeof(poison));
    qtest_memwrite(qts, BUF_BASE, poison, sizeof(poison));

    uint8_t big[BUF1_CAP + 200];
    make_frame(big, sizeof(big), 0x7e);
    inject(fd, big, sizeof(big));
    pump(qts);

    /* Descriptor still owned by the DMA (not consumed), buffer untouched. */
    g_assert_cmphex(desc_ctl(qts, 0) & RDES0_OWN, ==, RDES0_OWN);
    uint8_t check[BUF_STRIDE + 64];
    qtest_memread(qts, BUF_BASE, check, sizeof(check));
    for (size_t i = 0; i < sizeof(check); i++) {
        g_assert_cmpuint(check[i], ==, 0xA5);          /* no byte written */
    }

    /* A normal frame now lands in descriptor 0 (still the current one). */
    uint8_t f[64];
    make_frame(f, sizeof(f), 0x33);
    inject(fd, f, sizeof(f));
    g_assert_true(wait_consumed(qts, 0));
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x33);
    close(fd);
    qtest_quit(qts);
}

/*
 * Two frames arrive with the ring full. Both must be held and then delivered in
 * order as descriptors are freed one at a time - none dropped, none reordered.
 * (The socket backend serialises them via read_poll, but the observable
 * contract is the same: hold all, deliver in order.)
 */
static void test_rx_two_held(void)
{
    QTestState *qts;
    int fd = start_qemu(&qts);
    setup_ring(qts);

    uint8_t f[64];
    for (int i = 0; i < NUM_DESC; i++) {
        make_frame(f, sizeof(f), 0x20 + i);
        inject(fd, f, sizeof(f));
        g_assert_true(wait_consumed(qts, i));
    }

    /* Ring full. Two more frames: both must be held. */
    make_frame(f, sizeof(f), 0x61);
    inject(fd, f, sizeof(f));
    pump(qts);
    make_frame(f, sizeof(f), 0x62);
    inject(fd, f, sizeof(f));
    pump(qts);
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x20);   /* nothing delivered yet */

    /* Free descriptor 0: the first held frame (0x61) lands there. */
    write_desc(qts, DESC_BASE, RDES0_OWN, BUF1_CAP, BUF_BASE,
               DESC_BASE + DESC_SIZE);
    qtest_writel(qts, DMA_STATUS, DMA_STATUS_RU);
    qtest_writel(qts, DMA_RCV_POLL_DMD, 1);
    g_assert_true(wait_consumed(qts, 0));
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x61);
    g_assert_cmpuint(buf_tag(qts, 1), ==, 0x21);   /* 0x62 not here yet */

    /* Free descriptor 1: the second held frame (0x62) lands there, in order. */
    write_desc(qts, DESC_BASE + DESC_SIZE, RDES0_OWN, BUF1_CAP,
               BUF_BASE + BUF_STRIDE, DESC_BASE + 2 * DESC_SIZE);
    qtest_writel(qts, DMA_STATUS, DMA_STATUS_RU);
    qtest_writel(qts, DMA_RCV_POLL_DMD, 1);
    g_assert_true(wait_consumed(qts, 1));
    g_assert_cmpuint(buf_tag(qts, 1), ==, 0x62);
    close(fd);
    qtest_quit(qts);
}

/* A frame exactly the buffer size is accepted; one byte larger is dropped. */
static void test_rx_size_boundary(void)
{
    QTestState *qts;
    int fd = start_qemu(&qts);
    setup_ring(qts);

    /* Exactly cap bytes: fits, delivered into descriptor 0. */
    uint8_t ok[BUF1_CAP];
    make_frame(ok, sizeof(ok), 0x41);
    inject(fd, ok, sizeof(ok));
    g_assert_true(wait_consumed(qts, 0));
    g_assert_cmpuint(buf_tag(qts, 0), ==, 0x41);

    /* cap + 1 bytes: dropped. Poison descriptor 1's buffer and check it is
       untouched and the descriptor stays owned by the DMA. */
    uint8_t poison[128];
    memset(poison, 0xA5, sizeof(poison));
    qtest_memwrite(qts, BUF_BASE + BUF_STRIDE, poison, sizeof(poison));

    uint8_t big[BUF1_CAP + 1];
    make_frame(big, sizeof(big), 0x42);
    inject(fd, big, sizeof(big));
    pump(qts);
    g_assert_cmphex(desc_ctl(qts, 1) & RDES0_OWN, ==, RDES0_OWN);
    uint8_t check[128];
    qtest_memread(qts, BUF_BASE + BUF_STRIDE, check, sizeof(check));
    for (size_t i = 0; i < sizeof(check); i++) {
        g_assert_cmpuint(check[i], ==, 0xA5);
    }

    /* A normal frame still lands in descriptor 1. */
    uint8_t f[64];
    make_frame(f, sizeof(f), 0x43);
    inject(fd, f, sizeof(f));
    g_assert_true(wait_consumed(qts, 1));
    g_assert_cmpuint(buf_tag(qts, 1), ==, 0x43);
    close(fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/mini404/eth_rx/basic", test_rx_basic);
    qtest_add_func("/mini404/eth_rx/backpressure", test_rx_backpressure);
    qtest_add_func("/mini404/eth_rx/two_held", test_rx_two_held);
    qtest_add_func("/mini404/eth_rx/oversize", test_rx_oversize);
    qtest_add_func("/mini404/eth_rx/size_boundary", test_rx_size_boundary);

    return g_test_run();
}
