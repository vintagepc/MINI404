from stm32_regtypes import STM32Chip, STM32Fixups, Register

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
    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        chip.periph_map["PWR"]["WUSCR"].fields.pop("CWUF")
        chip.periph_map["PWR"]["WUCR"].fields.pop("WUPEN")

    @staticmethod
    def do_custom_gen(chip: STM32Chip):
        chip.generate_meta_only("CRC")
        chip.generate_meta_only("RNG")
