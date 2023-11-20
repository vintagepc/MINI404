/*
    TMC2130 - TMC2130 simulation heavily based on MK404's TMC2130.

    Adapted for Mini404 in 2020-3 by VintagePC <https://github.com/vintagepc/>

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
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "migration/vmstate.h"
#include "../utility/macros.h"
#include "../utility/p404_motor_if.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"
#include <math.h>

//#define DEBUG_TMC2130 1

#ifdef DEBUG_TMC2130
#define DPRINTF(fmt, ...) \
do { printf("tmc2130: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "tmc2130: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "tmc2130: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

// the internal programming registers.
typedef union
{
    uint32_t raw[128]; // There are 128, 7-bit addressing.
    // Add fields for specific ones down the line as you see fit...
    struct {
        struct {
            uint8_t I_scale_analog  :1;
            uint8_t internal_Rsense :1;
            uint8_t en_pwm_mode :1;
            uint8_t enc_communication   :1;
            uint8_t shaft   :1;
            uint8_t diag0_error :1;
            uint8_t diag0_optw  :1;
            uint8_t diag0_stall :1;
            uint8_t diag1_stall :1;
            uint8_t diag1_index :1;
            uint8_t diag1_onstate   :1;
            uint8_t diag1_steps_skipped :1;
            uint8_t diag0_int_pushpull  :1;
            uint8_t diag1_int_pushpull  :1;
            uint8_t small_hysteresis    :1;
            uint8_t stop_enable :1;
            uint8_t direct_mode         :1;
            uint16_t :14;
        }  QEMU_PACKED GCONF;             // 0x00
        struct                 // 0x01
        {
            uint8_t reset   :1;
            uint8_t drv_err :1;
            uint8_t uv_cp   :1;
            uint32_t :29; // unused
        }  QEMU_PACKED GSTAT;
        uint32_t _unimplemented[106]; //0x02 - 0x6B
        struct                        //0x6C
        {
            uint32_t toff		:4;
            uint32_t hstrt		:3;
            uint32_t hend		:4;
            uint32_t fd3		:1;
            uint32_t disfdcc	:1;
            uint32_t rndtf		:1;
            uint32_t chm		:1;
            uint32_t tbl		:2;
            uint32_t vsense		:1;
            uint32_t vhighfs	:1;
            uint32_t vhighchm	:1;
            uint32_t sync		:4;
            uint32_t mres		:4;
            uint32_t intpol		:1;
            uint32_t dedge		:1;
            uint32_t diss2g		:1;
            uint32_t			:1;
        } QEMU_PACKED CHOPCONF;
        uint32_t _unimplemented2[2]; //0x6D - 0x6E
        struct                       //0x6F
        {
            uint16_t SG_RESULT   :10;
            uint8_t             :5;
            uint8_t fsactive    :1;
            uint8_t CS_ACTUAL   :5;
            uint8_t             :3;
            uint8_t stallGuard  :1;
            uint8_t ot          :1;
            uint8_t otpw        :1;
            uint8_t sg2a        :1;
            uint8_t sg2b        :1;
            uint8_t ola         :1;
            uint8_t olb         :1;
            uint8_t stst        :1;
        }  QEMU_PACKED DRV_STATUS;
    }defs;
} tmc2130_registers_t;


typedef union {
    uint64_t all :40;
    struct {
        uint32_t data :32; // 32 bits of data
        uint8_t address :7;
        uint8_t RW :1;
    } QEMU_PACKED bitsIn;
    struct {
        uint32_t data :32; // 32 bits of data
        uint8_t reset_flag :1;
        uint8_t driver_error :1;
        uint8_t sg2 :1;
        uint8_t standstill :1;
        uint8_t :4; // unused
    }  QEMU_PACKED bitsOut;
    uint8_t bytes[5]; // Raw bytes as piped in/out by SPI.
} tmc2130_cmd_t;

typedef struct tmc2130_state {

    SSIPeripheral ssidev;

    unsigned char id;

    bool dir;

    bool enabled;

    bool last_step;

    bool stalled;

    bool stealthmode;

    uint8_t is_inverted;

    int64_t current_step;
    uint64_t max_step;
    uint16_t ms_increment;
    uint32_t max_steps_per_mm;   // Maximum number of steps/mm at full microstep resolution.

    float current_position; // Current position, in mm.

    tmc2130_registers_t regs; // The programming registers.

    tmc2130_cmd_t cmd_in, cmd_proc, cmd_out; // the previous data for output.

	bool diag_state;

	bool ext_stall;
    qemu_irq irq_diag;
	qemu_irq stall_indicator; // For indicator, regardless of polarity of DIAG pin.
    qemu_irq hard_out;
    qemu_irq position_out, um_out;

	qemu_irq peek; // Needed for SWSPI to "peek" at next byte.

    QEMUTimer *standstill;

	p404_motorif_status_t vis;

} tmc2130_state;

enum {
    ActGetPosFloat,
    ActWaitUntilInsideZoneMM,
    ActWaitUntilOutsideZoneMM,
	ActSetStall,
	ActToggleStall,
};

#define TYPE_TMC2130 "tmc2130"
OBJECT_DECLARE_SIMPLE_TYPE(tmc2130_state, TMC2130);
OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(tmc2130_state, tmc2130, TMC2130, SSI_PERIPHERAL,{TYPE_P404_SCRIPTABLE}, {TYPE_P404_MOTOR_IF}, {NULL});

static void tmc2130_check_raise_diag(tmc2130_state *s, int32_t value){
    bool bDiag = s->regs.defs.GCONF.diag0_stall || s->regs.defs.GCONF.diag1_stall;
    //printf("%c Diag: %01x SG %d PP %01x %01x \n",m_cAxis.load(), bDiag, s->regs.defs.DRV_STATUS.stallGuard, s->regs.defs.GCONF.diag0_int_pushpull, value );
    if (bDiag)
	{
		bool output = value == s->regs.defs.GCONF.diag0_int_pushpull;
		if (output ^ s->diag_state || output)
		{
			qemu_set_irq(s->stall_indicator,value);
        	qemu_set_irq(s->irq_diag,output);
			s->diag_state = output;
		}
	}
}

// External stall helper for corexy...
static void tmc2130_ext_stall(void *opaque, int n, int level) {
    tmc2130_state *s = opaque;
	s->ext_stall = level;
}

static void tmc2130_enable(void *opaque, int n, int level) {
    tmc2130_state *s = opaque;
    s->enabled = (level==0);
	s->vis.status.enabled = s->enabled;
    s->vis.status.changed |= true;
    if (level==1)
    {
        tmc2130_check_raise_diag(s, 0); // EN H clears diag.
    }
}

static float tmc2130_step_to_pos(int32_t step, uint32_t max_steps_per_mm)
{
	return (float)(step)/(float)(max_steps_per_mm);
}

// int32_t tmc2130_pos_to_step(float pos, uint32_t max_steps_per_mm)
// {
// 	return pos*(float)(max_steps_per_mm); // Convert pos to steps, we always work in the full 256 microstep workspace.
// }

static bool tmc2130_pos_in_range(P404ScriptIF *obj, unsigned int action, script_args args){
    tmc2130_state *s = TMC2130(obj);
    float target = scripthost_get_float(args, 0);
    float margin = scripthost_get_float(args, 1);
    return (s->current_position < target - margin / 2 || s->current_position > target + margin / 2);
}

static int tmc2130_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    tmc2130_state *s = TMC2130(obj);
    switch (action)
    {
        case ActGetPosFloat:
            script_print_float(s->current_position);
            break;
        case ActWaitUntilInsideZoneMM:
            if(tmc2130_pos_in_range(obj, action, args))
                return ScriptLS_Waiting;
            break;
        case ActWaitUntilOutsideZoneMM:
            if(!tmc2130_pos_in_range(obj, action, args))
                return ScriptLS_Waiting;
            break;
		case ActToggleStall:
				s->stalled^=true;
        		s->regs.defs.DRV_STATUS.SG_RESULT = s->stalled? 0 : 250;
				tmc2130_check_raise_diag(s, s->stalled);
			break;
		case ActSetStall:
				s->stalled=scripthost_get_bool(args, 0);
        		s->regs.defs.DRV_STATUS.SG_RESULT = s->stalled? 0 : 250;
				tmc2130_check_raise_diag(s, s->stalled);
			break;
        default:
            return ScriptLS_Unhandled;

    }
    return ScriptLS_Finished;
}

// Fired on standstill. set the STST flag and clear DIAG
static void tmc2130_standstill_timer(void *opaque)
{
    tmc2130_state *s = opaque;
    s->regs.defs.DRV_STATUS.stst = 1;
    tmc2130_check_raise_diag(s, 0);
    s->regs.defs.DRV_STATUS.SG_RESULT = 250;
    s->vis.status.stalled = false;
}

static void tmc2130_step(void *opaque, int n, int value) {

    tmc2130_state *s = opaque;
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
        s->last_step = value;
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

    s->current_position = tmc2130_step_to_pos(s->current_step, s->max_steps_per_mm);
	s->vis.current_pos = s->current_position;
    s->vis.status.changed |= true;
    qemu_set_irq(s->position_out, s->current_step);
    qemu_set_irq(s->um_out, s->current_position*1000.f);
	// The above may call out and cascade which alters the ext_stall flag
	bStall |= s->stalled  || s->ext_stall;
	s->vis.status.stalled = bStall;
    s->vis.status.changed |= true;
    if (bStall)
    {
        if (s->current_step==0) qemu_set_irq(s->hard_out,1);
        //qemu_set_irq(s->irq_diag,1);
		tmc2130_check_raise_diag(s, 1);
        s->regs.defs.DRV_STATUS.SG_RESULT = 0;
    }
    else if (!bStall)
    {
        qemu_set_irq(s->hard_out,0);
        //qemu_set_irq(s->irq_diag,0);
		tmc2130_check_raise_diag(s, 0);
        s->regs.defs.DRV_STATUS.SG_RESULT  = 250;
    }
    s->regs.defs.DRV_STATUS.stst = false;
    // 2^20 comes from the datasheet.

    // Internal clock is 12 MHz, 2^20 cycles is ~87 msec.
    timer_mod(s->standstill, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+87);
}

static void tmc2130_dir(void *opaque, int n, int level) {
    tmc2130_state *s = opaque;
    s->dir = (level^s->is_inverted)&0x1;
    s->vis.status.dir = s->dir;
}

static void tmc2130_create_reply(tmc2130_state *s) {
    s->cmd_out.all = 0x00; // Copy over.
    if (s->cmd_proc.bitsIn.RW == 0) // Last in was a read.
    {
        s->cmd_out.bitsOut.data = s->regs.raw[s->cmd_proc.bitsIn.address];
        if (s->cmd_proc.bitsIn.address == 0x01)
		{
            s->regs.raw[0x01] = 0; // GSTAT is cleared after read.
		}
        // TRACE(printf("Reading out %02x (%10lx)\n", s->cmd_proc.bitsIn.address, s->cmd_out.bitsOut.data));
    }
    else
	{
        s->cmd_out.bitsOut.data = s->cmd_proc.bitsOut.data;
	}
    // If the last was a write, the old data is left intact.

    // Set the status bits on the reply:
    s->cmd_out.bitsOut.driver_error = s->regs.defs.GSTAT.drv_err;
    s->cmd_out.bitsOut.reset_flag = s->regs.defs.GSTAT.reset;
    s->cmd_out.bitsOut.sg2 = s->regs.defs.DRV_STATUS.stallGuard;
    s->cmd_out.bitsOut.standstill = s->regs.defs.DRV_STATUS.stst;
    //(printf("Reply built: %10lx\n",m_cmdOut.all));
}

static void tmc2130_process_cmd(tmc2130_state *s) {
    // TRACE(printf("tmc2130 %c cmd: w: %x a: %02x  d: %08lx\n",m_cAxis.load(), m_cmdProc.bitsIn.RW, m_cmdProc.bitsIn.address, m_cmdProc.bitsIn.data));
    if (s->cmd_proc.bitsIn.RW)
    {
        s->regs.raw[s->cmd_proc.bitsIn.address] = s->cmd_proc.bitsIn.data;
        //printf("REG %c %02x set to: %010x\n", m_cAxis, m_cmdIn.bitsIn.address, m_cmdIn.bitsIn.data);
		switch (s->cmd_proc.bitsIn.address)
		{
			case 0x00: // GCONF
				tmc2130_check_raise_diag(s, s->regs.defs.DRV_STATUS.stallGuard); // Adjust DIAG out, it mayhave  been reconfigured.
				s->stealthmode = s->regs.defs.GCONF.en_pwm_mode;
				break;
			case 0x6C: // Chopconf
				s->ms_increment = pow(2,s->regs.defs.CHOPCONF.mres);
				break;
		}
    }
    else
    {
        // TRACE(printf("Read command on register: %02x\n", s->cmd_proc.bitsIn.address));
    }
    tmc2130_create_reply(s);
}

static uint32_t tmc2130_transfer(SSIPeripheral *dev, uint32_t data)
{
    tmc2130_state *s = TMC2130(dev);

    s->cmd_in.all<<=8; // Shift bits up
    s->cmd_in.bytes[0] = data & 0xFFU;
    //TRACE(printf("TMC2130 %c: byte received: %02x (%010lx)\n",m_cAxis.load(),value, m_cmdIn.all));
    // Clock out a reply byte, MSB first
    uint8_t byte = s->cmd_out.bytes[4];
    s->cmd_out.all<<=8;
	qemu_set_irq(s->peek, s->cmd_out.bytes[4]); // notify next byte.
    //TRACE(printf("TMC2130 %c: Clocking (%10lx) out %02x\n",m_cAxis.load(),m_cmdOut.all,byte));
    return byte; // SPIPeripheral takes care of the reply.
}

static int tmc2130_cs_changed(SSIPeripheral *dev, bool select) {
    // TRACE(printf("TMC2130 %c: CSEL changed to %02x\n",m_cAxis.load(),value));
    if (select) // Just finished a CSEL
    {
        tmc2130_state *s = TMC2130(dev);
        s->cmd_proc = s->cmd_in;
        tmc2130_process_cmd(s);
    }
    return 0;
}

static const p404_motorif_status_t* tmc2130_get_status(P404MotorIF* p)
{
    tmc2130_state *s = TMC2130(p);
    return &s->vis;
}

static void tmc2130_finalize(Object *obj){
}

static void tmc2130_realize(SSIPeripheral *obj, Error **errp){
    tmc2130_state *s = TMC2130(obj);
    const char buffer[2] = {s->id, '\0'};
    script_handle pScript = script_instance_new(P404_SCRIPTABLE(s), &buffer[0]);
    script_register_action(pScript, "GetPosFloat","Reports current position in mm.",ActGetPosFloat);
    script_register_action(pScript, "WaitUntilInsideZoneMM","Wait until the axis reaches the requested absolute position +/- margin (mm,mm).",ActWaitUntilInsideZoneMM);
    script_add_arg_float(pScript,ActWaitUntilInsideZoneMM);
    script_add_arg_float(pScript,ActWaitUntilInsideZoneMM);
    script_register_action(pScript, "WaitUntilOutsideZoneMM","Wait until the axis moves away from the requested absolute position +/- margin (mm,mm).",ActWaitUntilOutsideZoneMM);
    script_add_arg_float(pScript,ActWaitUntilOutsideZoneMM);
    script_add_arg_float(pScript,ActWaitUntilOutsideZoneMM);
	script_register_action(pScript, "ToggleStall", "Toggles the stallguard value.", ActToggleStall);
	script_register_action(pScript, "SetStall", "Sets the stallguard value as specified.", ActSetStall);
	script_add_arg_bool(pScript, ActSetStall);
    scripthost_register_scriptable(pScript);
	s->vis.label = s->id;
    s->vis.max_pos = (float)s->max_step/(float)s->max_steps_per_mm;
    s->vis.status.changed = true;
}

static void tmc2130_init(Object *obj){

    tmc2130_state *s = TMC2130(obj);
    s->id=' ';
    s->dir = 0;
    s->ms_increment = 16;
    s->is_inverted = 0;
    // s->max_steps_per_mm = 256*100;
    // s->max_step  = 160*16*100;
    s->current_step = 10 * s->ms_increment; // 10mm
    s->regs.defs.DRV_STATUS.SG_RESULT  = 250;
    s->regs.defs.DRV_STATUS.stst = 1;

    qdev_init_gpio_in_named( DEVICE(obj),tmc2130_dir, "dir",1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2130_step, "step",1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2130_enable, "enable",1);
    qdev_init_gpio_in_named( DEVICE(obj),tmc2130_ext_stall, "ext-stall",1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->irq_diag, "diag", 1);
	qdev_init_gpio_out_named(DEVICE(obj),&s->stall_indicator, "stall-indicator", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->hard_out, "hard", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->position_out, "step-out", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->um_out, "um-out", 1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->peek, "spi-peek", 1);
    // qemu_set_irq(s->irq_diag,0);
	tmc2130_check_raise_diag(s, 0);
    qemu_set_irq(s->hard_out,0);

    s->standstill =
        timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)tmc2130_standstill_timer, s);

}

static Property tmc2130_properties[] = {
    DEFINE_PROP_UINT8("axis", tmc2130_state, id,(uint8_t)' '),
    DEFINE_PROP_UINT8("inverted",tmc2130_state, is_inverted, 0),
    DEFINE_PROP_UINT64("max_step", tmc2130_state, max_step,16*100),
    DEFINE_PROP_UINT32("fullstepspermm",tmc2130_state, max_steps_per_mm,160*16*100),
    DEFINE_PROP_END_OF_LIST()
};


static int tmc2130_post_load(void *opaque, int version_id)
{
    tmc2130_state *s = TMC2130(opaque);
    s->current_position = tmc2130_step_to_pos(s->current_step, s->max_steps_per_mm);

    return 0;
}

static const VMStateDescription vmstate_tmc2130 = {
    .name = TYPE_TMC2130,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = tmc2130_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_SSI_PERIPHERAL(ssidev,tmc2130_state),
        VMSTATE_UINT32_ARRAY(regs.raw, tmc2130_state,128),
        VMSTATE_UINT8_ARRAY(cmd_in.bytes, tmc2130_state,5),
        VMSTATE_UINT8_ARRAY(cmd_proc.bytes, tmc2130_state,5),
        VMSTATE_UINT8_ARRAY(cmd_out.bytes, tmc2130_state,5),
        VMSTATE_UINT8(id,tmc2130_state),
        VMSTATE_BOOL(dir,tmc2130_state),
        VMSTATE_BOOL(enabled,tmc2130_state),
        VMSTATE_BOOL(last_step,tmc2130_state),
        VMSTATE_BOOL(stalled,tmc2130_state),
        VMSTATE_BOOL(stealthmode,tmc2130_state),
        VMSTATE_UINT8(is_inverted, tmc2130_state),
        VMSTATE_INT64(current_step,tmc2130_state),
        VMSTATE_UINT64(max_step,tmc2130_state),
        VMSTATE_UINT16(ms_increment,tmc2130_state),
        VMSTATE_UINT32(max_steps_per_mm,tmc2130_state),
        VMSTATE_TIMER_PTR(standstill,tmc2130_state),
        VMSTATE_END_OF_LIST(),
    }
};

static void tmc2130_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_tmc2130;

    device_class_set_props(dc, tmc2130_properties);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    k->realize = tmc2130_realize;
    k->transfer = tmc2130_transfer;
    k->set_cs = tmc2130_cs_changed;
    k->cs_polarity = SSI_CS_LOW;

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = tmc2130_process_action;

	P404MotorIFClass *mc = P404_MOTOR_IF_CLASS(klass);
    mc->get_current_status = tmc2130_get_status;
}
