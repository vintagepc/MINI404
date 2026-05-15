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
        # FDCAN_Global and FDCAN_Config are separate typedefs in the header;
        # merge them into a single FDCAN peripheral as they represent one
        # logical block (per-instance registers + shared config).
        chip.periph_map["FDCAN"] = chip.periph_map.pop("FDCAN_Global")
        chip.periph_map["FDCAN"]["CKDIV"] = chip.periph_map["FDCAN_Config"]["CKDIV"]
        chip.periph_map.pop("FDCAN_Config")

        rcc = chip.periph_map["RCC"]
        rcc["ICSCR"].unimplemented = True

        # EXTICR[4] is auto-expanded by the parser from the array declaration.
        # IT_LINE_SR[32] is also auto-expanded, but the HAL array name differs
        # from the datasheet naming (ITLINE0..31).  Rename here so that the
        # bitfield defines (SYSCFG_ITLINE{N}_SR_*) are captured in phase 2.
        syscfg = chip.periph_map["SYSCFG"]
        for i in range(32):
            old_name = f"IT_LINE_SR{i + 1}"
            if old_name in syscfg:
                reg = syscfg.pop(old_name)
                reg.name = f"ITLINE{i}"
                syscfg[f"ITLINE{i}"] = reg

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
        for reg in "CFGR2", "CFGR3":
            syscfg[reg].unimplemented = True
        # ITLINE0..31 were renamed from IT_LINE_SR1..32 in post_register_fixups so
        # the bitfield parser could capture SYSCFG_ITLINE{N}_SR_* defines.
        # Strip the SR_ prefix the HAL uses (removeprefix is literal, unlike lstrip)
        # and mark all registers unimplemented.
        for i in range(32):
            reg = syscfg[f"ITLINE{i}"]
            reg.unimplemented = True
            new_fields = {}
            for fname, field in reg.fields.items():
                new_name = fname.removeprefix("SR_")
                field.name = new_name
                new_fields[new_name] = field
            reg.fields = new_fields
    
