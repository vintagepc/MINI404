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
    
