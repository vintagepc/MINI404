/*
 * ST25DV64K dynamic NFC/RFID tag + EEPROM -- "system" register page
 *
 * The ST25DV64K exposes two separate I2C addresses: the bulk EEPROM /
 * dynamic-register "user memory" interface (modeled by a stock
 * at24c-eeprom instance, since firmware only ever reads/writes it as
 * plain memory), and this small "system configuration" register bank,
 * reached at a second I2C address (datasheet: 0x57 / write 0xAE, read
 * 0xAF).
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


#define TYPE_ST25DV64K_SYSPAGE "st25dv64k-syspage"

/* System configuration register addresses (ST25DV64K datasheet, system area) */
enum {
    REG_GPO         = 0x00,
    REG_IT_TIME     = 0x01,
    REG_EH_MODE     = 0x02,
    REG_RF_MNGT     = 0x03,
    REG_RFA1SS      = 0x04,
    REG_ENDA1       = 0x05,
    REG_RFA2SS      = 0x06,
    REG_ENDA2       = 0x07,
    REG_RFA3SS      = 0x08,
    REG_ENDA3       = 0x09,
    REG_RFA4SS      = 0x0A,
    REG_I2CSS       = 0x0B,
    REG_LOCK_CCFILE = 0x0C,
    REG_MB_MODE     = 0x0D,
    REG_MB_WDG      = 0x0E,
    REG_LOCK_CFG    = 0x0F,
};

#define ST25DV64K_NUM_REGS 0x10

/*
 * The "present I2C password" mechanism lives at register address 0x0900,
 * far outside the config bank above: a fixed 17-byte message (8-byte
 * password, a validation byte, then the password again). Firmware
 * (st25dv64k_present_pwd()) presents the factory-default all-zero
 * password before every config-register write. This model has nothing
 * to gate behind a password, so it simply ACKs and discards any write
 * outside REG_GPO..REG_LOCK_CFG -- which already covers this address
 * correctly without needing to special-case it.
 */
#define ST25DV64K_REG_I2C_PWD 0x0900

typedef struct QEMU_PACKED {
    uint8_t gpo;          /* 0x00 */
    uint8_t it_time;      /* 0x01 */
    uint8_t eh_mode;      /* 0x02 */
    uint8_t rf_mngt;      /* 0x03 */
    uint8_t rfa1ss;       /* 0x04 */
    uint8_t enda1;        /* 0x05 */
    uint8_t rfa2ss;       /* 0x06 */
    uint8_t enda2;        /* 0x07 */
    uint8_t rfa3ss;       /* 0x08 */
    uint8_t enda3;        /* 0x09 */
    uint8_t rfa4ss;       /* 0x0A */
    uint8_t i2css;        /* 0x0B */
    uint8_t lock_ccfile;  /* 0x0C */
    uint8_t mb_mode;      /* 0x0D */
    uint8_t mb_wdg;       /* 0x0E */
    uint8_t lock_cfg;     /* 0x0F */
} ST25DV64KSysRegDefs_t;

typedef union {
    uint8_t raw[ST25DV64K_NUM_REGS];
    ST25DV64KSysRegDefs_t defs;
} ST25DV64KSysRegs_t;

QEMU_BUILD_BUG_MSG(sizeof(ST25DV64KSysRegDefs_t) != ST25DV64K_NUM_REGS,
                   "ST25DV64K syspage register bank size mismatch");
QEMU_BUILD_BUG_MSG(offsetof(ST25DV64KSysRegDefs_t, lock_ccfile) != REG_LOCK_CCFILE,
                   "ST25DV64K syspage LOCK_CCFILE register offset mismatch");
QEMU_BUILD_BUG_MSG(offsetof(ST25DV64KSysRegDefs_t, lock_cfg) != REG_LOCK_CFG,
                   "ST25DV64K syspage LOCK_CFG register offset mismatch");

typedef struct ST25DV64KSysPageState {
    I2CSlave parent_obj;

    uint16_t reg_addr;
    uint8_t addr_phase;  /* 0-2: number of register-address bytes received */

    ST25DV64KSysRegs_t regs;
} ST25DV64KSysPageState;

DECLARE_INSTANCE_CHECKER(ST25DV64KSysPageState, ST25DV64K_SYSPAGE, TYPE_ST25DV64K_SYSPAGE)

static void st25dv64k_syspage_reset_regs(ST25DV64KSysPageState *s)
{
    memset(&s->regs, 0, sizeof(s->regs));
}

static int st25dv64k_syspage_event(I2CSlave *ss, enum i2c_event event)
{
    ST25DV64KSysPageState *s = ST25DV64K_SYSPAGE(ss);
    if (event == I2C_START_SEND) {
        s->addr_phase = 0;
    }
    return 0;
}

static uint8_t st25dv64k_syspage_recv(I2CSlave *ss)
{
    ST25DV64KSysPageState *s = ST25DV64K_SYSPAGE(ss);
    if (s->reg_addr >= ST25DV64K_NUM_REGS) {
        return 0xFF;
    }

    uint8_t data = s->regs.raw[s->reg_addr];
    s->reg_addr++;
    return data;
}

static int st25dv64k_syspage_send(I2CSlave *ss, uint8_t data)
{
    ST25DV64KSysPageState *s = ST25DV64K_SYSPAGE(ss);

    if (s->addr_phase < 2) {
        if (s->addr_phase == 0) {
            s->reg_addr = (uint16_t)data << 8;
        } else {
            s->reg_addr |= data;
        }
        s->addr_phase++;
        return 0;
    }

    if (s->reg_addr < ST25DV64K_NUM_REGS) {
        s->regs.raw[s->reg_addr] = data;
    }
    /*
     * Writes past the modeled config bank -- including the 17-byte
     * password-presentation message at 0x0900 -- are silently ACKed and
     * discarded for now
     */
    s->reg_addr++;
    return 0;
}

static void st25dv64k_syspage_realize(DeviceState *dev, Error **errp)
{
    ST25DV64KSysPageState *s = ST25DV64K_SYSPAGE(dev);
    st25dv64k_syspage_reset_regs(s);
}

static void st25dv64k_syspage_reset(DeviceState *dev)
{
    ST25DV64KSysPageState *s = ST25DV64K_SYSPAGE(dev);
    st25dv64k_syspage_reset_regs(s);
}

static
void st25dv64k_syspage_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = &st25dv64k_syspage_realize;
    k->recv = &st25dv64k_syspage_recv;
    k->send = &st25dv64k_syspage_send;
    k->event = &st25dv64k_syspage_event;

    device_class_set_legacy_reset(dc, st25dv64k_syspage_reset);
}

static
const TypeInfo st25dv64k_syspage_type = {
    .name = TYPE_ST25DV64K_SYSPAGE,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ST25DV64KSysPageState),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = st25dv64k_syspage_class_init,
};

static void st25dv64k_syspage_register(void)
{
    type_register_static(&st25dv64k_syspage_type);
}

type_init(st25dv64k_syspage_register)
