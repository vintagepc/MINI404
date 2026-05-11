from stm32_regtypes import *

class stm32c092xx(STM32Fixups):
    @staticmethod
    def supplemental_data(chip: STM32Chip):
        crc = chip.periph_map["CRC"]
        crc["DR"].reset_value = 0xFFFFFFFF
        crc["INIT"].reset_value = 0xFFFFFFFF
        crc["POL"].reset_value = 0x04C11DB7
        for field in crc["CR"].fields.values():
            if field.name not in ["RESET"]:
                field.unimplemented = True

        rcc = chip.periph_map["RCC"]
        rcc["CR"].reset_value = 0b1010101000000
        rcc["AHBENR"].reset_value = 0x100
        rcc["IOPSMENR"].reset_value = rcc["IOPSMENR"].get_valid_mask()
        rcc["AHBSMENR"].reset_value = rcc["AHBSMENR"].get_valid_mask()
        rcc["APBSMENR2"].reset_value = rcc["APBSMENR2"].get_valid_mask()

        flash = chip.periph_map["FLASH"]
        flash["ACR"].reset_value = 1 << 18 | 1 << 9
        flash["CR"].reset_value = 0xC0000000
        for reg in "PCROP1BSR","PCROP1BER","PCROP1AER","PCROP1ASR":
            flash[reg].unimplemented = True
    
        exti = chip.periph_map["EXTI"]
        exti["IMR1"].reset_value = 0xFFF80000
        for reg in "IMR1", "EMR1":
            exti[reg].unimplemented = True




    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        chip.periph_map["FDCAN"] = chip.periph_map.pop("FDCAN_Global")
        chip.periph_map["FDCAN"]["CKDIV"] = chip.periph_map["FDCAN_Config"]["CKDIV"]
        chip.periph_map.pop("FDCAN_Config")

        rcc = chip.periph_map["RCC"]
        rcc["ICSCR"].unimplemented = True

        syscfg = chip.periph_map["SYSCFG"]
        # Add entries for SYSCFG_ITLINE0-31:
        for i in range(32):
            name = f"ITLINE{i}"
            address = 0x80 + (i * 4)
            syscfg[name] = Register(name=name, desc="Interrupt line status register", hex_addr= "".join(hex(address)) , int_addr=address, fields={}, access=None, reset_value=0)

        exti = chip.periph_map["EXTI"]
        for i in range(4):
            name = f"EXTICR{1+i}"
            address = 0x60 + (i * 4)
            exti[name] = Register(name=name, desc="External interrupt configuration register", hex_addr= "".join(hex(address)) , int_addr=address, fields={}, access=None, reset_value=0)

        can = chip.periph_map["FDCAN"]
        can["CREL"].reset_value = 0x32141218
        can["ENDN"].reset_value = 0x87654321
        can["DBTP"].reset_value = 0xA33
        can["CCCR"].reset_value = 0x1
        can["NBTP"].reset_value = 0x06000A03
        can["TOCC"].reset_value = 0xFFFF0000
        can["TOCV"].reset_value = 0x0000FFFF
        can["PSR"].reset_value = 0x0707
        can["XIDAM"].reset_value = 0x1FFFFFFF
        can["TXFQS"].reset_value = 0x00000003


    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        syscfg = chip.periph_map["SYSCFG"]
        for field in syscfg["CFGR1"].fields.values():
            if field.name not in ["MEM_MODE"]:
                field.unimplemented = True
        for reg in "CFGR2","CFGR3":
            syscfg[reg].unimplemented = True
        for i in range(32):
            name = f"ITLINE{i}"
            reg = syscfg[name]
            reg.unimplemented = True
            for field in reg.fields.values():
                field.name = field.name.lstrip("SR_")
    
