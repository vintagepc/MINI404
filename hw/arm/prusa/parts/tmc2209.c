/*
 * TMC2209 - heavily based on MK404 TMC2130.
 *
 * This code is licensed under the GPL.
 */

/* The controller can support a variety of different displays, but we only
   implement one.  Most of the commends relating to brightness and geometry
   setup are ignored. */

#include "qemu/osdep.h"
#include "hw/irq.h"
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
        struct 
        {
            uint8_t ifcnt :8;
            uint32_t :24;
        } __attribute__ ((__packed__)) IFCNT;  // 0x02
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

typedef union { 
    uint64_t raw;
    uint32_t dwords[2];
    struct {
        uint8_t crc;

    }
    // struct {
    //     uint64_t crc :8;
    //     uint64_t data :32;
    //     uint64_t rw :1;
    //     uint64_t address :7;
    //     uint64_t slave :8;
    //     uint64_t sync :8;
    // }  __attribute__ ((__packed__)) write;
    // struct {
    //     uint64_t sync :8;
    //     uint64_t slave :8;
    //     uint64_t address :7;
    //     uint64_t rw :1;
    //     uint64_t crc :8;
    //     uint64_t :32;
    // }  __attribute__ ((__packed__)) read;
} tmc2209_cmd;


struct tmc2209_state {

    SysBusDevice parent_obj;
    SSISlave i2cdev;

    uint8_t address;
    char id;
    uint64_t cmdIn; 
    uint8_t bytePos;

    bool dir;

    bool enabled;

    bool last_step;

    bool stalled;

    int64_t current_step; 
    uint64_t max_step;
    uint16_t ms_increment;
    uint32_t max_steps_per_mm;   // Maximum number of steps/mm at full microstep resolution.

    float current_position; // Current position, in mm.

    tmc2209_registers_t regs; // The programming registers. 

    qemu_irq irq_diag;
    
};

#define TYPE_TMC2209 "tmc2209"
OBJECT_DECLARE_SIMPLE_TYPE(tmc2209_state, TMC2209)

static uint32_t tmc2209_transfer(SSISlave *dev, uint32_t data)
{
    tmc2209_state *s = TMC2209(dev);
    // printf("DATA: %llx\n",data);
    if (s->bytePos==0)
    {
        s->cmdIn = (uint64_t)data<<32;
        s->bytePos++;
        return 0;
    } else {
        s->cmdIn |= data;
        s->bytePos = 0;
    }
    //printf("Cmd: %llx\n",s->cmdIn);
    tmc2209_cmd cmd;
    // cmd.raw = s->cmdIn;
    // printf("%02x %02x %d to %02x with data %08lx (%02x)\n", cmd.write.sync, cmd.write.slave, cmd.write.rw, cmd.write.address, cmd.write.data, cmd.write.crc);
    // TODO... reply. 

    return 0;

}

static void tmc2209_enable(void *opaque, int n, int level) {
    TMC2209(opaque)->enabled = (level==0);
}

static float tmc2209_step_to_pos(int32_t step, uint32_t max_steps_per_mm)
{
	return (float)(step)/(float)(max_steps_per_mm);
}

int32_t tmc2209_pos_tostep(float pos, uint32_t max_steps_per_mm)
{
	return pos*(float)(max_steps_per_mm); // Convert pos to steps, we always work in the full 256 microstep workspace.
}

// static void tmc2209_check_raise_diag(tmc2209_state *s, int32_t value){
//     bool bDiag = s->regs.defs.GCONF.diag0_stall || s->regs.defs.GCONF.diag1_stall;
//     //printf("%c Diag: %01x SG %d PP %01x %01x \n",m_cAxis.load(), bDiag, s->regs.defs.DRV_STATUS.stallGuard, s->regs.defs.GCONF.diag0_int_pushpull, value );
//     if (bDiag)
// 	{
// 		//printf("Raised: %d\n", value ==  s->regs.defs.GCONF.diag0_int_pushpull);
//         qemu_set_irq(s->irq_diag, value == s->regs.defs.GCONF.diag0_int_pushpull);
// 	}
// }

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
    // CancelTimer(m_fcnStandstill,this);
    if (s->dir)
	{
        s->current_step-=s->ms_increment;
	}
    else
	{
        s->current_step+=s->ms_increment;
	}
    bool bStall = false;
    if (1)//!cfg.bHasNoEndStops)
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
    uint32_t posOut;
	// std::memcpy (&posOut, &m_fCurPos, 4);
    // RaiseIRQ(POSITION_OUT, posOut);
	// RaiseIRQ(STEP_POS_OUT, s->current_step);
    // TRACE(printf("cur pos: %f (%u)\n",s->current_position,s->current_step));
	bStall |= s->stalled;
    if (bStall)
    {
        qemu_set_irq(s->irq_diag,1);
        s->regs.defs.SG_RESULT.sg_result = 0;
    }
    else if (!bStall)
    {
            qemu_set_irq(s->irq_diag,0);
          s->regs.defs.SG_RESULT.sg_result = 250;
    }
    s->regs.defs.DRV_STATUS.stst = false;
    // 2^20 comes from the datasheet.
   // RegisterTimer(m_fcnStandstill,1U<<20U,this);
}

static void tmc2209_dir(void *opaque, int n, int level) {
    TMC2209(opaque)->dir = level;
}

static void tmc2209_check_diag() {

}
static void tmc2209_realize(SSISlave *dev, Error **errp){

    tmc2209_state *s = TMC2209(dev);
    s->id=' ';
    s->address=0;
    s->bytePos = 0;
    s->cmdIn = 0;

    qdev_init_gpio_in_named(DEVICE(dev),tmc2209_dir, "tmc2209-dir",1);
    qdev_init_gpio_in_named(DEVICE(dev),tmc2209_step, "tmc2209-step",1);
    qdev_init_gpio_in_named(DEVICE(dev),tmc2209_enable, "tmc2209-enable",1);
    qdev_init_gpio_out_named(DEVICE(dev),&s->irq_diag, "tmc2209-diag", 1);
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
};

static void tmc22092_register_types(void)
{
    type_register_static(&tmc2209_info);
}

type_init(tmc22092_register_types)
;
