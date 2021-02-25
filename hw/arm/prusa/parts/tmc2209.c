/*
    TMC2209 - TMC2209 simulation heavily based on MK404's TMC2130.
	
    Adapted for Mini404 in 2020 by VintagePC <https://github.com/vintagepc/>

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
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "chardev/char-fe.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"

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

#define TMC2209_CMD_LEN 8

// the internal programming registers.
typedef union 
{
    uint32_t raw[128]; // There are 128, 7-bit addressing.
    // Add fields for specific ones down the line as you see fit...
    struct {
        struct {
            uint8_t I_scale_analog  :1;
            uint8_t internal_Rsense :1;
            uint8_t en_spreadcycle :1;
            uint8_t shaft   :1;
            uint8_t index_otpw :1;
            uint8_t index_step  :1;
            uint8_t pdn_disable :1;
            uint8_t mstep_reg_select :1;
            uint8_t multistep_filt :1;
            uint8_t test_mode   :1;
            uint32_t :22;
        }  __attribute__ ((__packed__)) GCONF;             // 0x00
        struct                 // 0x01
        {
            uint8_t reset   :1;
            uint8_t drv_err :1;
            uint8_t uv_cp   :1;
            uint32_t :29; // unused
        }  __attribute__ ((__packed__)) GSTAT; // 0x01
        struct                 // 0x01
        {
            uint32_t ifcnt :8;
            uint32_t :24; // unused
        }  __attribute__ ((__packed__)) IFCNT; // 0x02
        uint32_t _unimplemented[61]; //0x03 - 0x6B
        struct // 0x40
        {
            uint32_t sgthrs :8;
            uint32_t :24;
        } __attribute__((__packed__)) SGTHRS; // 0x40
        struct // 0x41
        {
            uint32_t sg_result :10;
            uint32_t :22;
        } __attribute__((__packed__)) SG_RESULT; // 0x41
        uint32_t _unimplemented2a[42]; //0x03 - 0x6B
        struct                        //0x6C
        {
            uint32_t toff		:4;
            uint32_t hstrt		:3;
            uint32_t hend		:4;
            uint32_t _unused	:4;
            uint32_t tbl    	:2;
            uint32_t vsense		:1;
            uint32_t _unus2		:6;
            uint32_t mres		:4;
            uint32_t intpol		:1;
            uint32_t dedge		:1;
            uint32_t diss2g		:1;
            uint32_t diss2vs	:1;
        } __attribute__ ((__packed__)) CHOPCONF;
        uint32_t _unimplemented2[2]; //0x6D - 0x6E
        struct                       //0x6F
        {
            uint8_t otpw        :1;
            uint8_t ot          :1;
            uint8_t sg2a        :1;
            uint8_t sg2b        :1;
            uint8_t s2vsa        :1;
            uint8_t s2vsb        :1;
            uint8_t ola         :1;
            uint8_t olb         :1;
            uint8_t t120         :1;
            uint8_t t143         :1;
            uint8_t t150         :1;
            uint8_t t157         :1;
            uint8_t _unused      :4;
            uint8_t cs_actual   :5;
            uint16_t _unus2      :9;
            uint8_t stealth     :1;
            uint8_t stst        :1;
        }  __attribute__ ((__packed__)) DRV_STATUS;
    }defs;
} tmc2209_registers_t;


struct tmc2209_state {

    SysBusDevice parent_obj;

    uint8_t rx_buffer[TMC2209_CMD_LEN];
    uint8_t rx_pos;

    uint8_t address;
    unsigned char id;

    uint64_t cmdIn; 
    uint8_t bytePos;

    bool dir;

    bool enabled;

    bool last_step;

    bool stalled;
    
    uint8_t is_inverted; 

    int64_t current_step; 
    uint64_t max_step;
    uint16_t ms_increment;
    uint32_t max_steps_per_mm;   // Maximum number of steps/mm at full microstep resolution.

    float current_position; // Current position, in mm.

    tmc2209_registers_t regs; // The programming registers. 

    qemu_irq irq_diag;
    qemu_irq hard_out;
    qemu_irq byte_out;
    qemu_irq position_out;

    QEMUTimer *standstill;
    
};

enum {
    ActGetPosFloat
};

#define TYPE_TMC2209 "tmc2209"
OBJECT_DECLARE_SIMPLE_TYPE(tmc2209_state, TMC2209)

// Unceremoniously lifted directly from the TMC buddy firmware code. 
static uint8_t tmc2209_calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}

static void tmc2209_enable(void *opaque, int n, int level) {
    tmc2209_state *s = opaque;
    s->enabled = (level==0);
    if (level==1)
    {
        qemu_set_irq(s->irq_diag,0); // EN H clears diag.
    }
}

static float tmc2209_step_to_pos(int32_t step, uint32_t max_steps_per_mm)
{
	return (float)(step)/(float)(max_steps_per_mm);
}

// int32_t tmc2209_pos_to_step(float pos, uint32_t max_steps_per_mm)
// {
// 	return pos*(float)(max_steps_per_mm); // Convert pos to steps, we always work in the full 256 microstep workspace.
// }

// static void tmc2209_check_raise_diag(tmc2209_state *s, int32_t value){
//     bool bDiag = s->regs.defs.GCONF.diag0_stall || s->regs.defs.GCONF.diag1_stall;
//     //printf("%c Diag: %01x SG %d PP %01x %01x \n",m_cAxis.load(), bDiag, s->regs.defs.DRV_STATUS.stallGuard, s->regs.defs.GCONF.diag0_int_pushpull, value );
//     if (bDiag)
// 	{
// 		//printf("Raised: %d\n", value ==  s->regs.defs.GCONF.diag0_int_pushpull);
//         qemu_set_irq(s->irq_diag, value == s->regs.defs.GCONF.diag0_int_pushpull);
// 	}
// }

static int tmc2209_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    tmc2209_state *s = TMC2209(obj);
    switch (action)
    {
        case ActGetPosFloat:
            script_print_float(s->current_position);
            break;
        default:
            return ScriptLS_Unhandled;

    }
    return ScriptLS_Finished;
}

// Fired on standstill. set the STST flag and clear DIAG
static void tmc2209_standstill_timer(void *opaque)
{
    tmc2209_state *s = opaque;
    s->regs.defs.DRV_STATUS.stst = 1;
    qemu_set_irq(s->irq_diag,0);
    s->regs.defs.SG_RESULT.sg_result = 0;
}

static void tmc2209_step(void *opaque, int n, int value) {
    
    tmc2209_state *s = opaque;
    if (!s->enabled) return;
	if (!s->regs.defs.CHOPCONF.dedge)
	{
		// In normal mode only step on rising pulse
		if (!value) return;
	}
	else
	{
		// With DEDGE step on each value change
		if (value == s->last_step) return;
	}
    if (s->dir)
	{
        s->current_step-=s->ms_increment;
	}
    else
	{
        s->current_step+=s->ms_increment;
	}
    bool bStall = false;
    if (s->max_step != 0 )// If max_step ==0 it means endstopless, e.g. extruder.
    {
        if (s->current_step<0)
        {
            s->current_step = 0;
            bStall = true;
        }
        else if (s->current_step>s->max_step)
        {
            s->current_step = s->max_step;
            bStall = true;
        }
    }

    s->current_position = tmc2209_step_to_pos(s->current_step, s->max_steps_per_mm);
    qemu_set_irq(s->position_out, s->current_step);
	bStall |= s->stalled;
    if (bStall)
    {
        if (s->current_step==0) qemu_set_irq(s->hard_out,1);
        qemu_set_irq(s->irq_diag,1);
        s->regs.defs.SG_RESULT.sg_result = 0;
    }
    else if (!bStall)
    {
            qemu_set_irq(s->hard_out,0);
            qemu_set_irq(s->irq_diag,0);
          s->regs.defs.SG_RESULT.sg_result = 250;
    }
    s->regs.defs.DRV_STATUS.stst = false;
    // 2^20 comes from the datasheet.

    // Internal clock is 12 MHz, 2^20 cycles is ~87 msec.
    timer_mod(s->standstill, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+87);
}

static void tmc2209_dir(void *opaque, int n, int level) {
    tmc2209_state *s = opaque;
    s->dir = (level^s->is_inverted)&0x1;
}

static void tmc2209_write(tmc2209_state *s)
{
    uint32_t data = 0;
    for(int i=3; i<7; i++)
    {
        data<<=8;
        data |=s->rx_buffer[i];
    }
    s->regs.raw[s->rx_buffer[2]&0x7F] = data;
    // TODO- check CRC.
    s->regs.defs.IFCNT.ifcnt++;
    
}

static void tmc2209_read(tmc2209_state *s)
{
    // TODO, actually construct reply.
    uint32_t data = s->regs.raw[s->rx_buffer[2]];
    uint8_t reply[8] = {0x05, 0xFF, s->rx_buffer[2],data>>24,data>>16,data>>8,data,0x00};
    reply[7] = tmc2209_calcCRC(reply,7);
    for (int i=0; i<8; i++)
    {
        qemu_set_irq(s->byte_out, reply[i]); // 
    }
}

static void tmc2209_receive(void *opaque, int n, int level)
{
    tmc2209_state *s = (tmc2209_state *)opaque;

    s->rx_buffer[s->rx_pos] = (uint8_t)level;
    s->rx_pos++;

    if (s->rx_pos == 8 && s->rx_buffer[2] & 0x80)
    {
        if (s->rx_buffer[1] == s->address) tmc2209_write(s);
        s->rx_pos = 0;
    } else if (s->rx_pos == 4 && !(s->rx_buffer[2] & 0x80)) {
        if (s->rx_buffer[1] == s->address) tmc2209_read(s);
        s->rx_pos = 0;
    }
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(tmc2209_state, tmc2209, TMC2209, SYS_BUS_DEVICE,{TYPE_P404_SCRIPTABLE}, {NULL});

static void tmc2209_finalize(Object *obj){
}

static void tmc2209_realize(DeviceState *obj, Error **errp){
    tmc2209_state *s = TMC2209(obj);
    const char buffer[2] = {s->id, '\0'};
    script_handle pScript = script_instance_new(P404_SCRIPTABLE(s), &buffer[0]);
    script_register_action(pScript, "GetPosFloat","Reports current position in mm.",ActGetPosFloat);
    scripthost_register_scriptable(pScript);
}

static void tmc2209_init(Object *obj){

    tmc2209_state *s = TMC2209(obj);
    s->id=' ';
    s->address=0;
    s->bytePos = 0;
    s->cmdIn = 0;
    s->dir = 0;
    s->ms_increment = 16;
    s->is_inverted = 0;
    // s->max_steps_per_mm = 256*100;
    // s->max_step  = 160*16*100;
    s->current_step = 0 * s->ms_increment; // 10mm
    s->regs.defs.SG_RESULT.sg_result = 250;
    s->regs.defs.DRV_STATUS.stst = 1;

    qdev_init_gpio_in_named( DEVICE(obj),tmc2209_dir, "tmc2209-dir",1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2209_step, "tmc2209-step",1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2209_enable, "tmc2209-enable",1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->irq_diag, "tmc2209-diag", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->hard_out, "tmc2209-hard", 1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2209_receive, "tmc2209-byte-in",1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->byte_out, "tmc2209-byte-out", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->position_out, "tmc2209-step-out", 1);
    qemu_set_irq(s->irq_diag,0);
    qemu_set_irq(s->hard_out,0);

    s->standstill =
        timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)tmc2209_standstill_timer, s);

}

static Property tmc2209_properties[] = {
    DEFINE_PROP_UINT8("axis", tmc2209_state, id,(uint8_t)' '),
    DEFINE_PROP_UINT8("address", tmc2209_state, address,0),
    DEFINE_PROP_UINT8("inverted",tmc2209_state, is_inverted, 0),
    DEFINE_PROP_UINT64("max_step", tmc2209_state, max_step,16*100),
    DEFINE_PROP_UINT32("fullstepspermm",tmc2209_state, max_steps_per_mm,160*16*100),
    DEFINE_PROP_END_OF_LIST()
};


static void tmc2209_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, tmc2209_properties);
    dc->realize = tmc2209_realize;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = tmc2209_process_action;
}
