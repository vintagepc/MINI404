/*
 * QEMU SMBus EEPROM device
 *
 * Copyright (c) 2007 Arastra, Inc.
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
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/smbus_slave.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "st25dv64k.h"
#include "qom/object.h"

//#define DEBUG
#define TYPE_ST25DV64K "st25dv64k"

OBJECT_DECLARE_SIMPLE_TYPE(st25dv64kDevice, ST25DV64K)

#define ST25DV64K_SIZE 64*1024

struct st25dv64kDevice {
    SMBusDevice smbusdev;
    uint8_t data[ST25DV64K_SIZE];
//    uint8_t *init_data;
    uint16_t offset;
    bool accessed;
};

static uint8_t eeprom_receive_byte(SMBusDevice *dev)
{
    st25dv64kDevice *eeprom = ST25DV64K(dev);
    uint8_t *data = eeprom->data;
    uint8_t val = data[eeprom->offset++];

    eeprom->accessed = true;
#ifdef DEBUG
    printf("eeprom_receive_byte: addr=0x%02x val=0x%02x\n",
           dev->i2c.address, val);
#endif
    return val;
}

static int eeprom_write_data(SMBusDevice *dev, uint8_t *buf, uint8_t len)
{
    st25dv64kDevice *eeprom = ST25DV64K(dev);
    uint8_t *data = eeprom->data;

    eeprom->accessed = true;
#ifdef DEBUG
    printf("eeprom_write_byte: addr=0x%02x cmd=0x%02x val=0x%02x\n",
           dev->i2c.address, buf[0], buf[1]);
#endif
    if (len<2) return -1; // Addr too short, expects 16 bit address
    /* len is guaranteed to be > 0 */
    eeprom->offset = buf[0]<<8 | buf[1];
    buf+=2;
    len-=2;

    for (; len > 0; len--) {
        data[eeprom->offset] = *buf++;
        eeprom->offset = (eeprom->offset + 1) % ST25DV64K_SIZE;
    }

    return 0;
}

// static bool st25dv64k_vmstate_needed(void *opaque)
// {
//     MachineClass *mc = MACHINE_GET_CLASS(qdev_get_machine());
//     st25dv64kDevice *eeprom = opaque;

//     return (eeprom->accessed || smbus_vmstate_needed(&eeprom->smbusdev)) &&
//         !mc->smbus_no_migration_support;
// }

// static const VMStateDescription vmstate_st25dv64k = {
//     .name = "smbus-eeprom",
//     .version_id = 1,
//     .minimum_version_id = 1,
//     .needed = st25dv64k_vmstate_needed,
//     .fields      = (VMStateField[]) {
//         VMSTATE_SMBUS_DEVICE(smbusdev, st25dv64kDevice),
//         VMSTATE_UINT8_ARRAY(data, st25dv64kDevice, ST25DV64K_SIZE),
//         VMSTATE_UINT16(offset, st25dv64kDevice),
//         VMSTATE_BOOL(accessed, st25dv64kDevice),
//         VMSTATE_END_OF_LIST()
//     }
// };

/*
 * Reset the EEPROM contents to the initial state on a reset.  This
 * isn't really how an EEPROM works, of course, but the general
 * principle of QEMU is to restore function on reset to what it would
 * be if QEMU was stopped and started.
 *
 * The proper thing to do would be to have a backing blockdev to hold
 * the contents and restore that on startup, and not do this on reset.
 * But until that time, act as if we had been stopped and restarted.
 */
static void st25dv64k_reset(DeviceState *dev)
{
    st25dv64kDevice *eeprom = ST25DV64K(dev);

  //  memcpy(eeprom->data, eeprom->init_data, ST25DV64K_SIZE);
    eeprom->offset = 0;
}

static void st25dv64k_realize(DeviceState *dev, Error **errp)
{
    // st25dv64kDevice *eeprom = ST25DV64K(dev);

    st25dv64k_reset(dev);
    // if (eeprom->init_data == NULL) {
    //     error_setg(errp, "init_data cannot be NULL");
    // }
}

static void st25dv64k_finalize(Object *obj)
{
    printf("EEPROM finalize\n");
}

static void st25dv64k_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);

    // TODO - get rid of the smbus references and revert this to a plain i2c eeprom. 

    dc->realize = st25dv64k_realize;
    dc->reset = st25dv64k_reset;
    sc->receive_byte = eeprom_receive_byte;
    sc->write_data = eeprom_write_data;
    // dc->vmsd = &vmstate_st25dv64k;
    /* Reason: init_data */
    dc->user_creatable = false;
}

static const TypeInfo st25dv64k_info = {
    .name          = TYPE_ST25DV64K,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(st25dv64kDevice),
    .class_init    = st25dv64k_class_initfn,
    .instance_finalize = st25dv64k_finalize,
};

static void st25dv64k_register_types(void)
{
    type_register_static(&st25dv64k_info);
}

type_init(st25dv64k_register_types)

void st25dv64k_init_one(I2CBus *smbus, uint8_t address)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_ST25DV64K);
    qdev_prop_set_uint8(dev, "address", address);
    /* FIXME: use an array of byte or block backend property? */
   // ST25DV64K(dev)->init_data = eeprom_buf;
    qdev_realize(dev, (BusState *)smbus, &error_fatal);
}

// void st25dv64k_init(I2CBus *smbus,
//                        const uint8_t *eeprom_spd, int eeprom_spd_size)
// {
//     int i;
//      /* XXX: make this persistent */

//     assert(nb_eeprom <= 8);
//     uint8_t *eeprom_buf = g_malloc0(8 * ST25DV64K_SIZE);
//     if (eeprom_spd_size > 0) {
//         memcpy(eeprom_buf, eeprom_spd, eeprom_spd_size);
//     }

//     for (i = 0; i < nb_eeprom; i++) {
//         st25dv64k_init_one(smbus, 0x50 + i,
//                               eeprom_buf + (i * ST25DV64K_SIZE));
//     }
// }

// /* Generate SDRAM SPD EEPROM data describing a module of type and size */
// uint8_t *spd_data_generate(enum sdram_type type, ram_addr_t ram_size)
// {
//     uint8_t *spd;
//     uint8_t nbanks;
//     uint16_t density;
//     uint32_t size;
//     int min_log2, max_log2, sz_log2;
//     int i;

//     switch (type) {
//     case SDR:
//         min_log2 = 2;
//         max_log2 = 9;
//         break;
//     case DDR:
//         min_log2 = 5;
//         max_log2 = 12;
//         break;
//     case DDR2:
//         min_log2 = 7;
//         max_log2 = 14;
//         break;
//     default:
//         g_assert_not_reached();
//     }
//     size = ram_size >> 20; /* work in terms of megabytes */
//     sz_log2 = 31 - clz32(size);
//     size = 1U << sz_log2;
//     assert(ram_size == size * MiB);
//     assert(sz_log2 >= min_log2);

//     nbanks = 1;
//     while (sz_log2 > max_log2 && nbanks < 8) {
//         sz_log2--;
//         nbanks *= 2;
//     }

//     assert(size == (1ULL << sz_log2) * nbanks);

//     /* split to 2 banks if possible to avoid a bug in MIPS Malta firmware */
//     if (nbanks == 1 && sz_log2 > min_log2) {
//         sz_log2--;
//         nbanks++;
//     }

//     density = 1ULL << (sz_log2 - 2);
//     switch (type) {
//     case DDR2:
//         density = (density & 0xe0) | (density >> 8 & 0x1f);
//         break;
//     case DDR:
//         density = (density & 0xf8) | (density >> 8 & 0x07);
//         break;
//     case SDR:
//     default:
//         density &= 0xff;
//         break;
//     }

//     spd = g_malloc0(256);
//     spd[0] = 128;   /* data bytes in EEPROM */
//     spd[1] = 8;     /* log2 size of EEPROM */
//     spd[2] = type;
//     spd[3] = 13;    /* row address bits */
//     spd[4] = 10;    /* column address bits */
//     spd[5] = (type == DDR2 ? nbanks - 1 : nbanks);
//     spd[6] = 64;    /* module data width */
//                     /* reserved / data width high */
//     spd[8] = 4;     /* interface voltage level */
//     spd[9] = 0x25;  /* highest CAS latency */
//     spd[10] = 1;    /* access time */
//                     /* DIMM configuration 0 = non-ECC */
//     spd[12] = 0x82; /* refresh requirements */
//     spd[13] = 8;    /* primary SDRAM width */
//                     /* ECC SDRAM width */
//     spd[15] = (type == DDR2 ? 0 : 1); /* reserved / delay for random col rd */
//     spd[16] = 12;   /* burst lengths supported */
//     spd[17] = 4;    /* banks per SDRAM device */
//     spd[18] = 12;   /* ~CAS latencies supported */
//     spd[19] = (type == DDR2 ? 0 : 1); /* reserved / ~CS latencies supported */
//     spd[20] = 2;    /* DIMM type / ~WE latencies */
//                     /* module features */
//                     /* memory chip features */
//     spd[23] = 0x12; /* clock cycle time @ medium CAS latency */
//                     /* data access time */
//                     /* clock cycle time @ short CAS latency */
//                     /* data access time */
//     spd[27] = 20;   /* min. row precharge time */
//     spd[28] = 15;   /* min. row active row delay */
//     spd[29] = 20;   /* min. ~RAS to ~CAS delay */
//     spd[30] = 45;   /* min. active to precharge time */
//     spd[31] = density;
//     spd[32] = 20;   /* addr/cmd setup time */
//     spd[33] = 8;    /* addr/cmd hold time */
//     spd[34] = 20;   /* data input setup time */
//     spd[35] = 8;    /* data input hold time */

//     /* checksum */
//     for (i = 0; i < 63; i++) {
//         spd[63] += spd[i];
//     }
//     return spd;
// }
