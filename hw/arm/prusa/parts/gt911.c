/*
    gt911.c - Sim touchscreen controller for Mini404

	Copyright 2022-3 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qom/object.h"


#define TYPE_GT911 "gt911"

enum reg_info
{
	RI_BEGIN = 0x8000,
	RI_XMAX = 0x8048,
	RI_YMAX = 0x804A,
	RI_MOD_SW1 = 0x804D,
	RI_CFG_END = 0x8100,
	RI_PID = 0x8140,
	RI_XRES = 0x8146,
	RI_YRES = 0x8148,
	RI_VID = 0x814A,
	RI_TOUCH = 0x814E,
	RI_POINTS_BEGIN = 0x814F,
	RI_POINT2 = 0x8157,
	RI_END = 0x8178,
};

typedef struct QEMU_PACKED GTPoint_t
{
	uint8_t track_id;
	uint16_t x;
	uint16_t y;
	uint16_t size;
	uint8_t _reserved;
} GTPoint_t;


typedef union GTRegs_t
{
	uint8_t raw[RI_END-RI_BEGIN];
	struct {
		uint8_t config[RI_XMAX - RI_BEGIN]; //0x8000 - 0x8047 was 257
		uint16_t X_MAX;// 0x8048
		uint16_t Y_MAX;// 0x804A
		uint8_t TOUCH_NUM; //0x804C
		struct {
			uint8_t INT :2;
			uint8_t SITO :1;
			uint8_t X2Y :1;
			uint8_t STRETCH_RANK :2;
			uint8_t X_INV :1;
			uint8_t Y_INV :1;
		} QEMU_PACKED MOD_SW1; // 0x804D
		uint8_t cfg2[RI_CFG_END - RI_MOD_SW1];
		uint8_t _reserved[0x3F]; // 0x8101-0x813F
		uint32_t PID; // 0x8140
		uint16_t FW_VER;
		uint16_t X_RES;
		uint16_t Y_RES;
		uint8_t VID;
		uint8_t _reserved2[3];
		struct {
			uint8_t PTS :4;
			uint8_t HAVE_KEY :1;
			uint8_t PROX_VALID :1;
			uint8_t LARGE_DET :1;
			uint8_t BUFFER_STATUS :1;
		} QEMU_PACKED TOUCH;
		GTPoint_t PT1;
		GTPoint_t PT2;
		GTPoint_t PT3;
		GTPoint_t PT4;
		GTPoint_t PT5;
		uint8_t _reserved3;
	} QEMU_PACKED defs;
} GTRegs_t;

QEMU_BUILD_BUG_MSG(sizeof(GTPoint_t) != 8U, "GTPoint is not the correct size!");
QEMU_BUILD_BUG_MSG(offsetof(GTRegs_t, defs.config) != offsetof(GTRegs_t, raw), "Point alignment invalid");
QEMU_BUILD_BUG_MSG(offsetof(GTRegs_t, defs.PID) != offsetof(GTRegs_t, raw[RI_PID-RI_BEGIN]), "PID alignment invalid");
QEMU_BUILD_BUG_MSG(offsetof(GTRegs_t, defs.PT1) != offsetof(GTRegs_t, raw[RI_POINTS_BEGIN-RI_BEGIN]), "Point struct alignment invalid");
QEMU_BUILD_BUG_MSG(offsetof(GTRegs_t, defs.PT2) != offsetof(GTRegs_t, raw[RI_POINT2-RI_BEGIN]), "Point struct alignment invalid");

typedef struct GT911State {
    I2CSlave parent_obj;
	union {
		uint16_t address;
		uint8_t bytes[2];
	};
	GTRegs_t regs;
	uint8_t rx_level;
	qemu_irq interrupt;

	int16_t cursor[2];

	QEMUTimer *touch_scan;

} GT911State;

DECLARE_INSTANCE_CHECKER(GT911State, GT911, TYPE_GT911)

static int gt911_event(I2CSlave *ss, enum i2c_event event)
{
    GT911State *s = GT911(ss);
	if (event == I2C_START_SEND)
	{
		s->rx_level = 2;
	}
	return 0;
}

static uint8_t gt911_recv(I2CSlave *ss)
{
    GT911State *s = GT911(ss);
    uint8_t ret = 0;
	{
		switch (s->bytes[1])
		{
		case 0x80:
		case 0x81:
			if (s->address < RI_END)
				ret = s->regs.raw[s->address-RI_BEGIN];
			else
				ret = 0;
			s->address++;
			break;
		default:
			printf("GT911: invalid read!\n");
			ret = 0;
			break;
		}
	}
//	if (ret!=0) printf("GT911 tx %04x -> %02x\n", s->address-1,ret);
    return ret;
}

static int gt911_send(I2CSlave *ss, uint8_t data)
{
    GT911State *s = GT911(ss);
	//printf("GT911 rx %02x\n", data);
	if (s->rx_level>0)
	{
		s->bytes[--(s->rx_level)] = data;
		return 0;
	}
	uint16_t index = s->address - RI_BEGIN;
	switch (index)
	{
		case 0x000 ... 0x100:
			s->regs.raw[index] = data;
			s->address++;
			break;
		case RI_TOUCH - RI_BEGIN:
			s->regs.raw[RI_TOUCH-RI_BEGIN] = data;
			s->regs.defs.TOUCH.BUFFER_STATUS = 1;
			break;
		default:
			printf("ERR: GT911: invalid write to address %04x\n", s->address);
			break;

	}
    return 0;
}

static void gt911_scan(void *opaque)
{
	GT911State *s = GT911(opaque);
	qemu_set_irq(s->interrupt, 1);
	timer_mod(s->touch_scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
}

static void gt911_coords_in(void* opaque, int n, int level)
{
	GT911State *s = GT911(opaque);
	switch (n)
	{
	case 0:
		s->cursor[0] += level;
		s->cursor[0] = MIN(MAX(s->cursor[0],0),s->regs.defs.Y_MAX);
		s->regs.defs.PT1.y = s->regs.defs.MOD_SW1.Y_INV ? s->cursor[0] : (s->regs.defs.Y_MAX - s->cursor[0]);
		break;
	case 1:
		s->cursor[1] += level;
		s->cursor[1] = MIN(MAX(s->cursor[1],0),s->regs.defs.X_MAX);
		s->regs.defs.PT1.x = s->cursor[1];
		break;
	case 2:
		s->regs.defs.TOUCH.BUFFER_STATUS = 1;
		s->regs.defs.TOUCH.PTS = level;
		break;
	}
}

static void gt911_realize(DeviceState *dev, Error **errp)
{
	GT911State *s = GT911(dev);
	QEMU_BUILD_BUG_MSG(sizeof(s->regs.raw) != sizeof(s->regs.defs), "Register union misaligned!");
	qdev_init_gpio_out(dev, &s->interrupt, 1);
	qdev_init_gpio_in_named(dev, gt911_coords_in, "x_y_touch",3);
	s->touch_scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, gt911_scan, s);
	timer_mod(s->touch_scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
}

static void gt911_reset(DeviceState *dev)
{
	GT911State *s = GT911(dev);
	s->regs.defs.TOUCH.BUFFER_STATUS = 1;
}

static Property gt911_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static
void gt911_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = &gt911_realize;
    k->recv = &gt911_recv;
    k->send = &gt911_send;
	k->event = &gt911_event;

    device_class_set_props(dc, gt911_props);
    dc->reset = gt911_reset;
}

static
const TypeInfo gt911_type = {
    .name = TYPE_GT911,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GT911State),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = gt911_class_init,
};

static void gt911_register(void)
{
    type_register_static(&gt911_type);
}

type_init(gt911_register)
