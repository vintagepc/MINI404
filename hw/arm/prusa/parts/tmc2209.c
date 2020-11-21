/*
 * TMC2209 OLED controller with OSRAM Pictiva 128x64 display.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* The controller can support a variety of different displays, but we only
   implement one.  Most of the commends relating to brightness and geometry
   setup are ignored. */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"

//#define DEBUG_TMC2209 1

#ifdef DEBUG_TMC2209
#define DPRINTF(fmt, ...) \
do { printf("tmc2209: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "tmc2209: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "tmc2209: error: " fmt , ## __VA_ARGS__);} while (0)
#endif



typedef union { 
    uint8_t dwords[8];
    uint64_t raw;
} tmc2209_cmd;

struct tmc2209_state {

    SysBusDevice parent_obj;
    SSISlave i2cdev;

    uint8_t address;
    char id;
    tmc2209_cmd cmdIn;
    tmc2209_cmd cmdOut;
    uint8_t bytePos;
    
};

#define TYPE_TMC2209 "tmc2209"
OBJECT_DECLARE_SIMPLE_TYPE(tmc2209_state, TMC2209)

static uint32_t tmc2209_transfer(SSISlave *dev, uint32_t data)
{
    tmc2209_state *s = TMC2209(dev);
    s->cmdIn.dwords[s->bytePos++] = data;
    if (s->bytePos>7)
    {
        printf("TMC %c CMD: %08x\n", s->id, s->cmdIn.raw);
        s->bytePos = 0;
    }
    
    // TODO... reply. 

    return 0;

}


static void tmc2209_realize(SSISlave *dev, Error **errp){

    tmc2209_state *s = TMC2209(dev);
    s->id=' ';
    s->address=0;
    s->bytePos = 0;
    s->cmdIn.raw = 0;
    s->cmdOut.raw = 0;

}

static void tmc2209_class_init(ObjectClass *klass, void *data)
{
    // DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->realize = tmc2209_realize;
    k->transfer = tmc2209_transfer;
    k->cs_polarity = SSI_CS_HIGH;

   // dc->vmsd = &vmstate_tmc2209;
}

static const TypeInfo tmc2209_info = {
    .name          = TYPE_TMC2209,
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(tmc2209_state),
    .class_init    = tmc2209_class_init,
    .instance_init = tmc2209_realize,
};

static void tmc22092_register_types(void)
{
    type_register_static(&tmc2209_info);
}

type_init(tmc22092_register_types)
;
