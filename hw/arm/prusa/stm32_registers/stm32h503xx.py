from stm32_regtypes import *
from stm32_fixupspec import PeripheralFixup, RegisterFixup, apply_peripheral_fixup
from peripherals.adcc import split_adcc
from peripherals.crc import CRC_H503
from peripherals.fdcan import merge_fdcan
from peripherals.rng import RNG_TYPE_A, RNG_H503

_RCC = PeripheralFixup(registers={
    "CR":         RegisterFixup(reset_value=0x0000002B),
    "HSICFGR":    RegisterFixup(reset_value=0x00400000, unimplemented=True),
    "CRRCR":      RegisterFixup(unimplemented=True),
    "CSICFGR":    RegisterFixup(reset_value=0x00200000, unimplemented=True),
    "CFGR1":      RegisterFixup(fields_unimplemented=["STOPWUCK", "STOPKERWUCK", "MCO1PRE", "MCO1SEL", "MCO2PRE", "MCO2SEL"]),
    "PLL1CFGR":   RegisterFixup(fields_unimplemented=["PLL1RGE", "PLL1VCOSEL"]),
    "PLL2CFGR":   RegisterFixup(fields_unimplemented=["PLL2RGE", "PLL2VCOSEL"]),
    "PLL1DIVR":   RegisterFixup(reset_value=0x01010280),
    "PLL2DIVR":   RegisterFixup(reset_value=0x01010280),
    "AHB1ENR":    RegisterFixup(reset_value=0x90000100),
    "AHB2ENR":    RegisterFixup(reset_value=0x40000000),
    "AHB1LPENR":  RegisterFixup(reset_value=0xFFFFFFFF),
    "AHB2LPENR":  RegisterFixup(reset_value=0xFFFFFFFF),
    "APB1LLPENR": RegisterFixup(reset_value=0xFFFFFFFF),
    "APB1HLPENR": RegisterFixup(reset_value=0xFFFFFFFF),
    "APB2LPENR":  RegisterFixup(reset_value=0xFFFFFFFF),
    "APB3LPENR":  RegisterFixup(reset_value=0xFFFFFFFF, fields_unimplemented=["RTCAPBLPEN"]),
    "CCIPR2":     RegisterFixup(fields_unimplemented=["LPTIM1SEL", "LPTIM2SEL"]),
    "CCIPR3":     RegisterFixup(fields_unimplemented=["LPUART1SEL"]),
    "CCIPR5":     RegisterFixup(fields_unimplemented=["DACSEL", "FDCANSEL"]),
    "PRIVCFGR":   RegisterFixup(fields_unimplemented=["PRIV"]),
    "RSR":        RegisterFixup(reset_value=0x0C000000),
})

_PWR = PeripheralFixup(
    all_unimplemented=True,
    registers={
        "PMCR":   RegisterFixup(reset_value=0x0000000C),
        "VOSSR":  RegisterFixup(reset_value=0x00002008),
        "SCCR":   RegisterFixup(reset_value=0x00000100),
        "IORETR": RegisterFixup(reset_value=0x00010001),
        "WUSCR":  RegisterFixup(fields_remove=["CWUF"]),
        "WUCR":   RegisterFixup(fields_remove=["WUPEN"]),
    })

_ICACHE = PeripheralFixup(
    all_unimplemented=True,
    registers={
        "CR": RegisterFixup(reset_value=0x00000004),
    })

_DMA = PeripheralFixup(registers={
    "CSR":      RegisterFixup(reset_value=0x00000001),
    "CLLR":     RegisterFixup(unimplemented=True),
    "CBR2":     RegisterFixup(unimplemented=True),
    "CTR3":     RegisterFixup(unimplemented=True),
    "CTR2":     RegisterFixup(unimplemented=True),
    "CLBAR":    RegisterFixup(unimplemented=True),
    "PRIVCFGR": RegisterFixup(unimplemented=True),
    "CBR1":     RegisterFixup(fields_unimplemented_except=["BNDT"]),
    "CTR1":     RegisterFixup(fields_unimplemented_except=["SDW_LOG2", "DDW_LOG2", "SINC", "DINC"]),
    "CCR":      RegisterFixup(fields_unimplemented_except=["EN", "TCIE", "HTIE"]),
})

_FLASH = PeripheralFixup(
    all_unimplemented=True,
    registers={
        "ACR":   RegisterFixup(reset_value=0x00000013, unimplemented=False,
                               fields_unimplemented=["PRFTEN", "WRHIGHFREQ"]),
        "OPTCR": RegisterFixup(reset_value=0x00000001),
    })

_ADC = PeripheralFixup(
    renames={"TR3": "H503_TR3", "CFGR": "CFGR1", "SMPR1": "SMPR"},
    registers={
        "CR":       RegisterFixup(reset_value=0x20000000),
        "CFGR1":    RegisterFixup(reset_value=0x80000000),
        "TR1":      RegisterFixup(reset_value=0x0FFF0000, unimplemented=True),
        "TR2":      RegisterFixup(reset_value=0x00FF0000, unimplemented=True),
        "H503_TR3": RegisterFixup(reset_value=0x00FF0000, unimplemented=True),
        "JSQR":     RegisterFixup(unimplemented=True),
        "JDR1":     RegisterFixup(unimplemented=True),
        "JDR2":     RegisterFixup(unimplemented=True),
        "JDR3":     RegisterFixup(unimplemented=True),
        "JDR4":     RegisterFixup(unimplemented=True),
        "OFR1":     RegisterFixup(unimplemented=True),
        "OFR2":     RegisterFixup(unimplemented=True),
        "OFR3":     RegisterFixup(unimplemented=True),
        "OFR4":     RegisterFixup(unimplemented=True),
        "AWD2CR":   RegisterFixup(unimplemented=True),
        "AWD3CR":   RegisterFixup(unimplemented=True),
        "DIFSEL":   RegisterFixup(unimplemented=True),
        "CALFACT":  RegisterFixup(unimplemented=True),
        "OR":       RegisterFixup(unimplemented=True),
    })

_EXTI_PRE = PeripheralFixup(renames={
    "PRIVCFGR1": "PRIVENR1",
    "PRIVCFGR2": "PRIVENR2",
})

_FLASH_PRE = PeripheralFixup(renames={
    "NSCR":       "CR",
    "NSSR":       "SR",
    "NSKEYR":     "KEYR",
    "NSCCR":      "CCR",
    "OPTSR_CUR":  "OPTSR",
    "OPTSR2_CUR": "OPTSR2",
})

_EXTI = PeripheralFixup(registers={
    "IMR2": RegisterFixup(fields_remove=["IM", "IM_2", "IM_3"]),
    "EMR1": RegisterFixup(fields_remove=["EM"]),
    "IMR1": RegisterFixup(fields_remove=["IM", "IM_1", "IM_2", "IM_3"]),
    "EMR2": RegisterFixup(fields_remove=["EM", "EM_2", "EM_3"]),
})

class stm32h503xx(STM32Fixups):
    common_periph = {
        "CRC":   "CRC_TYPE_A",
        "I2C":   "I2C_TYPE_A",
        "IWDG":  "IWDG_TYPE_A",
        "ADCC":  "ADCC_TYPE_A",
        "FDCAN": "FDCAN_TYPE_A",
        "RNG":   "RNG_TYPE_A",
        "EXTI":  "EXTI_TYPE_A",
        "USART": "USART_TYPE_A",
    }

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        reg_map = chip.periph_map
        apply_peripheral_fixup(reg_map, "RCC", _RCC)
        apply_peripheral_fixup(reg_map, "PWR", _PWR)
        apply_peripheral_fixup(reg_map, "ICACHE", _ICACHE)

        usb = reg_map.pop("USB_DRD")
        reg_map["USB"] = usb
        reg_map["GPIO"]["AFRL"] = Register(name="AFRL", desc="GPIO alternate function low register", hex_addr="0x20", int_addr=0x20, fields={}, access=None, reset_value=0)
        reg_map["GPIO"]["AFRH"] = Register(name="AFRH", desc="GPIO alternate function High register", hex_addr="0x24", int_addr=0x24, fields={}, access=None, reset_value=0)

        apply_peripheral_fixup(reg_map, "CRC", CRC_H503)
        apply_peripheral_fixup(reg_map, "RNG", RNG_H503)
        apply_peripheral_fixup(reg_map, "RNG", RNG_TYPE_A)

        apply_peripheral_fixup(reg_map, "ADC", _ADC)
        apply_peripheral_fixup(reg_map, "DMA", _DMA)

        apply_peripheral_fixup(reg_map, "FLASH", _FLASH)
        # ST does not define _Pos/_Msk macros for key registers (the full 32-bit
        # width is a single implicit field).  Add the field explicitly so the
        # generated mask reflects that all bits are non-reserved.
        for reg_name in ["KEYR", "OPTKEYR"]:
            reg_map["FLASH"][reg_name].fields["KEY"] = RegisterBitField(
                name="KEY", desc="Key", shift=0, width=32, permissions=None, unimplemented=False)
        reg_map["FLASH"]["KEYR"].unimplemented = False

        apply_peripheral_fixup(reg_map, "EXTI", _EXTI)

    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        chip.periph_map["ADC"]["CCR"] = Register(name="CCR", desc="ADC common control register", hex_addr="0x0", int_addr=0x0, fields={}, access=None, reset_value=0)
        # Merge the DMA_Channel registers into the DMA periph.
        for i in chip.periph_map["DMA_Channel"]:
            chip.periph_map["DMA"][i] = chip.periph_map["DMA_Channel"][i]
        chip.periph_map.pop("DMA_Channel")
        apply_peripheral_fixup(chip.periph_map, "EXTI", _EXTI_PRE)
        apply_peripheral_fixup(chip.periph_map, "FLASH", _FLASH_PRE)
        merge_fdcan(chip.periph_map)

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        split_adcc(chip.periph_map)
