/*
 * LDC1612 2-Channel Inductance-to-Digital Converter
 *
 * Written for Mini404 in 2026 by VintagePC <https://github.com/vintagepc/>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "qom/object.h"


#define TYPE_LDC1612 "ldc1612"

#define LDC1612_NUM_REGS 0x80

/* Register addresses (SNOSCZ0A Table 1) */
enum {
    REG_DATA_CH0        = 0x00,
    REG_DATA_CH0_LSB    = 0x01,
    REG_DATA_CH1        = 0x02,
    REG_DATA_CH1_LSB    = 0x03,
    REG_RCOUNT_CH0      = 0x08,
    REG_RCOUNT_CH1      = 0x09,
    REG_OFFSET_CH0      = 0x0C,
    REG_OFFSET_CH1      = 0x0D,
    REG_SETTLECOUNT_CH0 = 0x10,
    REG_SETTLECOUNT_CH1 = 0x11,
    REG_CLOCK_DIV_CH0   = 0x14,
    REG_CLOCK_DIV_CH1   = 0x15,
    REG_STATUS          = 0x18,
    REG_ERROR_CONFIG    = 0x19,
    REG_CONFIG          = 0x1A,
    REG_MUX_CONFIG      = 0x1B,
    REG_RESET_DEV       = 0x1C,
    REG_DRIVE_CUR_CH0   = 0x1E,
    REG_DRIVE_CUR_CH1   = 0x1F,
    REG_MANUFACTURER_ID = 0x7E,
    REG_DEVICE_ID       = 0x7F,
};

#define LDC1612_MFR_ID  0x5449U
#define LDC1612_DEV_ID  0x3055U

/* DATA_CHx [15:12] = per-channel error flags, [11:0] = upper 12 bits of result */
typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t DATA_MSB : 12;
        uint16_t ERR_ZC   : 1;
        uint16_t ERR_OR   : 1;
        uint16_t ERR_UR   : 1;
        uint16_t ERR_AHV  : 1;
    };
} LDC1612DataReg_t;

typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t _res0    : 6;
        uint16_t DRDY     : 1;
        uint16_t _res1    : 1;
        uint16_t ERR_ZC   : 1;
        uint16_t ERR_ALV  : 1;
        uint16_t ERR_AHV  : 1;
        uint16_t ERR_WD   : 1;
        uint16_t ERR_OR   : 1;
        uint16_t ERR_UR   : 1;
        uint16_t ERR_CHAN : 2;
    };
} LDC1612StatusReg_t;

/* ERROR_CONFIG[8] = DRDY_2INT routes data-ready to the INTB pin */
typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t _res0      : 8;
        uint16_t DRDY_2INT  : 1;
        uint16_t _res1      : 1;
        uint16_t ZC_ERR2OUT : 1;
        uint16_t AL_ERR2OUT : 1;
        uint16_t AH_ERR2OUT : 1;
        uint16_t WD_ERR2OUT : 1;
        uint16_t OR_ERR2OUT : 1;
        uint16_t UR_ERR2OUT : 1;
    };
} LDC1612ErrorConfigReg_t;

typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t _res0        : 6;
        uint16_t HIGH_CUR_DRV : 1;
        uint16_t INTB_DIS     : 1;
        uint16_t _res1        : 2;
        uint16_t REF_CLK_SRC  : 1;
        uint16_t AUTO_AMP_DIS : 1;
        uint16_t SENSOR_ACT   : 1;
        uint16_t RP_OVERRIDE  : 1;
        uint16_t SLEEP_MODE   : 1;
        uint16_t ACTIVE_CHAN  : 1;  /* reflects currently converting channel */
    };
} LDC1612ConfigReg_t;

typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t DEGLITCH    : 3;
        uint16_t _res0       : 6;
        uint16_t RR_SEQUENCE : 3;
        uint16_t AUTOSCAN_EN : 1;
        uint16_t _res1       : 3;
    };
} LDC1612MuxConfigReg_t;

typedef union {
    uint16_t raw;
    struct QEMU_PACKED {
        uint16_t _res  : 15;
        uint16_t RESET : 1;
    };
} LDC1612ResetDevReg_t;

/* Flat register bank matching the device address map */
typedef struct QEMU_PACKED {
    LDC1612DataReg_t      data_ch0;          /* 0x00 */
    uint16_t              data_ch0_lsb;      /* 0x01 */
    LDC1612DataReg_t      data_ch1;          /* 0x02 */
    uint16_t              data_ch1_lsb;      /* 0x03 */
    uint16_t              _reserved0[4];     /* 0x04-0x07 */
    uint16_t              rcount_ch0;        /* 0x08 */
    uint16_t              rcount_ch1;        /* 0x09 */
    uint16_t              _reserved1[2];     /* 0x0A-0x0B */
    uint16_t              offset_ch0;        /* 0x0C */
    uint16_t              offset_ch1;        /* 0x0D */
    uint16_t              _reserved2[2];     /* 0x0E-0x0F */
    uint16_t              settlecount_ch0;   /* 0x10 */
    uint16_t              settlecount_ch1;   /* 0x11 */
    uint16_t              _reserved3[2];     /* 0x12-0x13 */
    uint16_t              clock_div_ch0;     /* 0x14 */
    uint16_t              clock_div_ch1;     /* 0x15 */
    uint16_t              _reserved4[2];     /* 0x16-0x17 */
    LDC1612StatusReg_t    status;            /* 0x18 */
    LDC1612ErrorConfigReg_t error_config;   /* 0x19 */
    LDC1612ConfigReg_t    config;            /* 0x1A */
    LDC1612MuxConfigReg_t mux_config;        /* 0x1B */
    LDC1612ResetDevReg_t  reset_dev;         /* 0x1C */
    uint16_t              _reserved5;        /* 0x1D */
    uint16_t              drive_cur_ch0;     /* 0x1E */
    uint16_t              drive_cur_ch1;     /* 0x1F */
    uint16_t              _reserved6[0x5E]; /* 0x20-0x7D */
    uint16_t              manufacturer_id;   /* 0x7E */
    uint16_t              device_id;         /* 0x7F */
} LDC1612RegDefs_t;

typedef union {
    uint16_t raw[LDC1612_NUM_REGS];
    LDC1612RegDefs_t defs;
} LDC1612Regs_t;

QEMU_BUILD_BUG_MSG(sizeof(LDC1612RegDefs_t) != LDC1612_NUM_REGS * sizeof(uint16_t),
                   "LDC1612 register bank size mismatch");
QEMU_BUILD_BUG_MSG(offsetof(LDC1612RegDefs_t, status) != REG_STATUS * sizeof(uint16_t),
                   "LDC1612 STATUS register offset mismatch");
QEMU_BUILD_BUG_MSG(offsetof(LDC1612RegDefs_t, config) != REG_CONFIG * sizeof(uint16_t),
                   "LDC1612 CONFIG register offset mismatch");
QEMU_BUILD_BUG_MSG(offsetof(LDC1612RegDefs_t, manufacturer_id) != REG_MANUFACTURER_ID * sizeof(uint16_t),
                   "LDC1612 MANUFACTURER_ID register offset mismatch");

typedef struct LDC1612State {
    I2CSlave parent_obj;

    uint8_t reg_addr;
    uint8_t tx_byte;  /* 0 = MSB phase, 1 = LSB phase */
    bool is_addr;     /* true when next send byte is the register pointer */

    qemu_irq interrupt;

    LDC1612Regs_t regs;
} LDC1612State;

DECLARE_INSTANCE_CHECKER(LDC1612State, LDC1612, TYPE_LDC1612)

static void ldc1612_reset_regs(LDC1612State *s)
{
    memset(&s->regs, 0, sizeof(s->regs));
    s->regs.defs.config.raw      = 0x2801;
    s->regs.defs.mux_config.raw  = 0x020C;
    s->regs.defs.manufacturer_id = LDC1612_MFR_ID;
    s->regs.defs.device_id       = LDC1612_DEV_ID;
}

/* INTB is active-low; asserted when DRDY is set, routes to INTB, and INTB is enabled */
static void ldc1612_update_irq(LDC1612State *s)
{
    bool active = s->regs.defs.status.DRDY &&
                  s->regs.defs.error_config.DRDY_2INT &&
                  !s->regs.defs.config.INTB_DIS;
    qemu_set_irq(s->interrupt, !active);
}

static bool ldc1612_is_readonly(uint8_t reg)
{
    switch (reg) {
    case REG_DATA_CH0:
    case REG_DATA_CH0_LSB:
    case REG_DATA_CH1:
    case REG_DATA_CH1_LSB:
    case REG_STATUS:
    case REG_MANUFACTURER_ID:
    case REG_DEVICE_ID:
        return true;
    default:
        return false;
    }
}

static void ldc1612_inject_data(LDC1612State *s, int ch, uint32_t val)
{
    if (ch == 0) {
        s->regs.defs.data_ch0.DATA_MSB = val >> 16;
        s->regs.defs.data_ch0_lsb = val & 0xFFFF;
    } else {
        s->regs.defs.data_ch1.DATA_MSB = val >> 16;
        s->regs.defs.data_ch1_lsb = val & 0xFFFF;
    }

    s->regs.defs.config.ACTIVE_CHAN = ch;
    s->regs.defs.status.DRDY = 1;
    ldc1612_update_irq(s);
}

static void ldc1612_data_in(void *opaque, int n, int level)
{
    LDC1612State *s = LDC1612(opaque);
    ldc1612_inject_data(s, n, (uint32_t)level);
}

static int ldc1612_event(I2CSlave *ss, enum i2c_event event)
{
    LDC1612State *s = LDC1612(ss);
    if (event == I2C_START_SEND) {
        s->is_addr = true;
        s->tx_byte = 0;
    } else if (event == I2C_START_RECV) {
        s->tx_byte = 0;
    }
    return 0;
}

static uint8_t ldc1612_recv(I2CSlave *ss)
{
    LDC1612State *s = LDC1612(ss);
    if (s->reg_addr >= LDC1612_NUM_REGS) {
        return 0;
    }

    uint16_t val = s->regs.raw[s->reg_addr];
    uint8_t data;
    if (s->tx_byte == 0) {
        data = val >> 8;
        s->tx_byte = 1;
    } else {
        data = val & 0xFF;
        s->tx_byte = 0;
        if (s->reg_addr == REG_STATUS) {
            s->regs.defs.status.DRDY = 0;
            ldc1612_update_irq(s);
        }
        s->reg_addr++;
    }
    return data;
}

static int ldc1612_send(I2CSlave *ss, uint8_t data)
{
    LDC1612State *s = LDC1612(ss);
    if (s->is_addr) {
        s->reg_addr = data;
        s->is_addr  = false;
        s->tx_byte  = 0;
        return 0;
    }

    if (s->reg_addr >= LDC1612_NUM_REGS) {
        return 1; /* NAK */
    }

    bool is_msb      = (s->tx_byte == 0);
    s->tx_byte      ^= 1;
    bool reg_complete = (s->tx_byte == 0);

    if (ldc1612_is_readonly(s->reg_addr)) {
        if (reg_complete) {
            s->reg_addr++;
        }
        return 0;
    }

    if (is_msb) {
        s->regs.raw[s->reg_addr] = (s->regs.raw[s->reg_addr] & 0x00FF) | ((uint16_t)data << 8);
    } else {
        s->regs.raw[s->reg_addr] = (s->regs.raw[s->reg_addr] & 0xFF00) | data;
        if (s->reg_addr == REG_RESET_DEV && s->regs.defs.reset_dev.RESET) {
            ldc1612_reset_regs(s);
            ldc1612_update_irq(s);
        } else if (s->reg_addr == REG_CONFIG || s->reg_addr == REG_ERROR_CONFIG) {
            ldc1612_update_irq(s);
        }
        s->reg_addr++;
    }
    return 0;
}

static void ldc1612_realize(DeviceState *dev, Error **errp)
{
    LDC1612State *s = LDC1612(dev);
    ldc1612_reset_regs(s);
    qdev_init_gpio_out(DEVICE(dev), &s->interrupt, 1);
    qdev_init_gpio_in_named(DEVICE(dev), ldc1612_data_in, "ch", 2);
    ldc1612_update_irq(s);
}

static void ldc1612_reset(DeviceState *dev)
{
    LDC1612State *s = LDC1612(dev);
    ldc1612_reset_regs(s);
    ldc1612_update_irq(s);
}

static
void ldc1612_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = &ldc1612_realize;
    k->recv = &ldc1612_recv;
    k->send = &ldc1612_send;
    k->event = &ldc1612_event;

    device_class_set_legacy_reset(dc, ldc1612_reset);
}

static
const TypeInfo ldc1612_type = {
    .name = TYPE_LDC1612,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LDC1612State),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = ldc1612_class_init,
};

static void ldc1612_register(void)
{
    type_register_static(&ldc1612_type);
}

type_init(ldc1612_register)
