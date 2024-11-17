from stm32_regtypes import *

class stm32h503xx(STM32Fixups):
    @staticmethod
    def supplemental_data(chip: STM32Chip):
        reg_map = chip.periph_map
        # These are direct from the datasheet(s):
        reg_map["RCC"]["CR"].reset_value = 0x0000002B
        reg_map["RCC"]["HSICFGR"].reset_value = 0x00400000
        reg_map["RCC"]["HSICFGR"].unimplemented = True
        reg_map["RCC"]["CRRCR"].unimplemented = True
        reg_map["RCC"]["CSICFGR"].reset_value = 0x00200000
        reg_map["RCC"]["CSICFGR"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["STOPWUCK"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["STOPKERWUCK"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["MCO1PRE"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["MCO1SEL"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["MCO2PRE"].unimplemented = True
        reg_map["RCC"]["CFGR1"].fields["MCO2SEL"].unimplemented = True
        reg_map["RCC"]["PLL1CFGR"].fields["PLL1RGE"].unimplemented = True
        reg_map["RCC"]["PLL2CFGR"].fields["PLL2RGE"].unimplemented = True
        reg_map["RCC"]["PLL1CFGR"].fields["PLL1VCOSEL"].unimplemented = True
        reg_map["RCC"]["PLL2CFGR"].fields["PLL2VCOSEL"].unimplemented = True
        reg_map["RCC"]["PLL1DIVR"].reset_value = 0x01010280
        reg_map["RCC"]["PLL2DIVR"].reset_value = 0x01010280
        reg_map["RCC"]["AHB1ENR"].reset_value = 0x90000100
        reg_map["RCC"]["AHB2ENR"].reset_value = 0x40000000
        reg_map["RCC"]["AHB1LPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["AHB2LPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["APB1LLPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["APB1HLPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["APB2LPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["APB3LPENR"].reset_value = 0xFFFFFFFF
        reg_map["RCC"]["APB3LPENR"].fields["RTCAPBLPEN"].unimplemented = True
        reg_map["RCC"]["CCIPR2"].fields["LPTIM1SEL"].unimplemented = True
        reg_map["RCC"]["CCIPR2"].fields["LPTIM2SEL"].unimplemented = True
        reg_map["RCC"]["CCIPR3"].fields["LPUART1SEL"].unimplemented = True
        reg_map["RCC"]["CCIPR5"].fields["DACSEL"].unimplemented = True
        reg_map["RCC"]["CCIPR5"].fields["FDCANSEL"].unimplemented = True
        reg_map["RCC"]["PRIVCFGR"].fields["PRIV"].unimplemented = True
        reg_map["RCC"]["RSR"].reset_value = 0x0C000000

        reg_map["PWR"]["PMCR"].reset_value = 0x0000000C
        reg_map["PWR"]["VOSSR"].reset_value = 0x00002008
        reg_map["PWR"]["SCCR"].reset_value = 0x00000100
        reg_map["PWR"]["IORETR"].reset_value = 0x00010001
        for k,v in reg_map["PWR"].items():
            v.unimplemented = True

        reg_map["ICACHE"]["CR"].reset_value = 0x00000004
        for k,v in reg_map["ICACHE"].items():
            v.unimplemented = True
        
        usb = reg_map.pop("USB_DRD")
        reg_map["USB"] = usb
        reg_map["GPIO"]["AFRL"] = Register(name = "AFRL", desc="GPIO alternate function low register", hex_addr = "0x20", int_addr = 0x20, fields = {}, access = None, reset_value = 0)
        reg_map["GPIO"]["AFRH"] = Register(name = "AFRH", desc="GPIO alternate function High register", hex_addr = "0x24", int_addr = 0x24, fields = {}, access = None, reset_value = 0)

        reg_map["CRC"]["DR"].reset_value = 0xFFFFFFFF
        reg_map["CRC"]["INIT"].reset_value = 0xFFFFFFFF
        reg_map["CRC"]["POL"].reset_value = 0x04C11DB7
        reg_map["CRC"].pop("HWCFGR")        
        reg_map["CRC"].pop("VERR")
        reg_map["CRC"].pop("PIDR")
        reg_map["CRC"].pop("SIDR")

        reg_map["RNG"]["CR"].reset_value = 0x0080D00
        reg_map["RNG"]["HTCR"].reset_value = 0x000072AC
        reg_map["RNG"]["DR"].fields["DR"] = RegisterBitField(name="DR", desc="Data register", shift=0, width=32, permissions=None, unimplemented=False)
        reg_map["RNG"]["HTCR"].unimplemented = True
        for k,v in reg_map["RNG"]["CR"].fields.items():
            if k != "RNGEN" and k != "IE":
                v.unimplemented = True
        reg_map["RNG"]["SR"].fields["SECS"].unimplemented = True
        reg_map["RNG"]["SR"].fields["SEIS"].unimplemented = True

        reg_map["ADC"]["CR"].reset_value = 0x20000000
        reg_map["ADC"]["CFGR"].reset_value = 0x80000000
        for i in ["TR1", "TR2", "TR3"]:
            chip.periph_map["ADC"][i].reset_value = 0x0FFF0000 if i == "TR1" else 0x00FF0000
            chip.periph_map["ADC"][i].unimplemented = True
        for i in ["JSQR", "JDR1", "JDR2", "JDR3", "JDR4", "OFR1", "OFR2", "OFR3", "OFR4", "AWD2CR","AWD3CR", "DIFSEL", "CALFACT", "OR"]:
            chip.periph_map["ADC"][i].unimplemented = True
        # Clarify TR3, it's not consistent depending on whether CHSELR is used:
        # Do some more renames too:
        for old,new in [["TR3", "H503_TR3"], ["CFGR", "CFGR1"], ["SMPR1", "SMPR"]]:
            chip.periph_map["ADC"][old].name = new
            chip.periph_map["ADC"][new] = chip.periph_map["ADC"].pop(old)
        # Set DMA defaults and extras:
        chip.periph_map["DMA"]["CSR"].reset_value = 0x00000001
        for i in ["CLLR", "CBR2", "CTR3", "CTR2", "CLBAR", "PRIVCFGR"]:
            chip.periph_map["DMA"][i].unimplemented = True
        for f in chip.periph_map["DMA"]["CBR1"].fields:
            if f != "BNDT":
                chip.periph_map["DMA"]["CBR1"].fields[f].unimplemented = True
        for f in chip.periph_map["DMA"]["CTR1"].fields:
            if f not in ["SDW_LOG2", "DDW_LOG2", "SINC", "DINC"]:
                chip.periph_map["DMA"]["CTR1"].fields[f].unimplemented = True
        for f in chip.periph_map["DMA"]["CCR"].fields:
            if f not in ["EN", "TCIE", "HTIE"]:
                chip.periph_map["DMA"]["CCR"].fields[f].unimplemented = True
        

    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        chip.periph_map["ADC"]["CCR"] = Register(name="CCR", desc="ADC common control register", hex_addr="0x0", int_addr=0x0, fields={}, access=None, reset_value=0)
        # Merge the DMA_Channel registers into the DMA periph:
        for i in chip.periph_map["DMA_Channel"]:
            chip.periph_map["DMA"][i] = chip.periph_map["DMA_Channel"][i]
        chip.periph_map.pop("DMA_Channel")

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        chip.periph_map["PWR"]["WUSCR"].fields.pop("CWUF")
        chip.periph_map["PWR"]["WUCR"].fields.pop("WUPEN")
        # Relocate the ADC CCR register to its own class for better overlap with how it's implemented in QEMU:
        chip.periph_map["ADCC"] = {}
        chip.periph_map["ADCC"]["CCR"] = chip.periph_map["ADC"]["CCR"]
        chip.periph_map["ADC"].pop("CCR")
