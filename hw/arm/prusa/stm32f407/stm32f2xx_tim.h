#ifndef STM32F2XX_TIM_H
#define STM32F2XX_TIM_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "stm32.h"

#define SHORT_REG_32(name,used) struct{ uint32_t name :used; uint32_t :32-used; } __attribute__ ((__packed__))

#define R_TIM_MAX    (0x54 / 4)

#define TYPE_STM32F4XX_TIMER "stm32f4xx-timer"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_tim, STM32F4XX_TIMER)

struct Stm32Rcc;

struct f2xx_tim {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    // Union-ized for my/code sanity and easier inspection during debugging. 
    union {
        uint32_t regs[R_TIM_MAX];
        struct {
            struct {
                uint32_t CEN :1;
                uint32_t UDIS :1;
                uint32_t URS :1;
                uint32_t OPM :1;
                uint32_t DIR :1;
                uint32_t CMS :2;
                uint32_t ARPE :1;
                uint32_t CKD :2;
                uint32_t :22; // unused.
            } __attribute__ ((__packed__)) CR1;
            struct {
                uint32_t CCPC :1;
                uint32_t _reserved :1;
                uint32_t CCUS :1;
                uint32_t CCDS :1;
                uint32_t MMS :3;
                uint32_t TI1S :1;
                uint32_t OIS1 :1;
                uint32_t OIS1N :1;
                uint32_t OIS2 :1;
                uint32_t OIS2N :1;
                uint32_t OIS3 :1;
                uint32_t OIS3N :1;
                uint32_t OIS4 :1;
                uint32_t :32-15; // unused.
            } __attribute__ ((__packed__)) CR2;         
            struct {
                uint32_t SMS :3;
                uint32_t _reserved :1;
                uint32_t TS :3;
                uint32_t MSM :1;
                uint32_t ETF :4;
                uint32_t ETPS :2;
                uint32_t ECE :1;
                uint32_t ETP :1;
                uint32_t :32-16; // unused.
            } __attribute__ ((__packed__)) SMCR;
            struct {
                uint32_t UIE :1;
                uint32_t CC1IE:1;
                uint32_t CC2IE:1;
                uint32_t CC3IE:1;
                uint32_t CC4IE:1;
                uint32_t COMIE:1;
                uint32_t TIE:1;
                uint32_t BIE:1;
                uint32_t UDE:1;
                uint32_t CC1DE:1;
                uint32_t CC2DE:1;
                uint32_t CC3DE:1;
                uint32_t CC4DE:1;
                uint32_t COMDE:1;
                uint32_t TDE:1;
                uint32_t :17;
            } __attribute__ ((__packed__))DIER;
            struct {
                uint32_t UIF:1;
                uint32_t CC1IF:1;
                uint32_t CC2IF:1;
                uint32_t CC3IF:1;
                uint32_t CC4IF:1;
                uint32_t COMIF:1;
                uint32_t TIF:1;
                uint32_t BIF:1;
                uint32_t _reserved :1;
                uint32_t CC1OF:1;
                uint32_t CC2OF:1;
                uint32_t CC3OF:1;
                uint32_t CC4OF:1;
                uint32_t :32-13;
            } __attribute__ ((__packed__))SR;
            struct {
                uint32_t UG:1;
                uint32_t CC1G:1;
                uint32_t CC2G:1;
                uint32_t CC3G:1;
                uint32_t CC4G:1;
                uint32_t COMG:1;
                uint32_t TG:1;
                uint32_t BG:1;
                uint32_t :32-8;
            } __attribute__ ((__packed__))EGR;
            struct {
                uint32_t CC1S :2;
                uint32_t OC1FE :1;
                uint32_t OC1PE :1;
                uint32_t OC1M :3;
                uint32_t OC1CE :1;
                uint32_t CC2S :2;
                uint32_t OC2FE :1;
                uint32_t OC2PE :1;
                uint32_t OC2M :3;
                uint32_t OC2CE :1;
                uint32_t :32-16;
            } __attribute__ ((__packed__))CCMR1; // N.B this is only the OC mode, not IC
            struct {
                uint32_t CC3S :2;
                uint32_t OC3FE :1;
                uint32_t OC3PE :1;
                uint32_t OC3M :3;
                uint32_t OC3CE :1;
                uint32_t CC4S :2;
                uint32_t OC4FE :1;
                uint32_t OC4PE :1;
                uint32_t OC4M :3;
                uint32_t OC4CE :1;
                uint32_t :32-16;
            } __attribute__ ((__packed__))CCMR2;
            struct {
                uint32_t CC1E :1;
                uint32_t CC1P :1;
                uint32_t CC1NE :1;
                uint32_t CC1NP :1;
                uint32_t CC2E :1;
                uint32_t CC2P :1;
                uint32_t CC2NE :1;
                uint32_t CC2NP :1;
                uint32_t CC3E :1;
                uint32_t CC3P :1;
                uint32_t CC3NE :1;
                uint32_t CC3NP :1;
                uint32_t CC4E :1;
                uint32_t CC4P :1;
                uint32_t :32-14;                
            } __attribute__ ((__packed__)) CCER;
            SHORT_REG_32(CNT, 16);
            SHORT_REG_32(PSC, 16);
            SHORT_REG_32(ARR, 16);
            SHORT_REG_32(REP,8) RCR;
            SHORT_REG_32(CCR1, 16);
            SHORT_REG_32(CCR2, 16);
            SHORT_REG_32(CCR3, 16);
            SHORT_REG_32(CCR4, 16);
            struct {
                uint32_t DT :8;
                uint32_t LOCK :2;
                uint32_t OSSI :1;
                uint32_t OSSR :1;
                uint32_t BKE :1;
                uint32_t BKP :1;
                uint32_t AOE :1;
                uint32_t MOE :1;
                uint32_t :32-16;
            } __attribute__ ((__packed__)) BDTR;
            struct {
                uint32_t DBA:5;
                uint32_t _reserved :3;
                uint32_t DBL :5;
                uint32_t :19;
            } __attribute__ ((__packed__))DCR;
            uint32_t DMAR;
            SHORT_REG_32(RMP,2) OR;
        } defs;
    };
    uint8_t id;
    qemu_irq pwm_ratio_changed[4];
    qemu_irq pwm_enable[4];

    stm32_periph_t periph;
    Stm32Rcc *rcc; // RCC for clock speed. 
};

#endif // STM32F2XX_TIM_H
