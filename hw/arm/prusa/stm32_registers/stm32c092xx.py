from stm32_regtypes import *
from stm32_fixupspec import PeripheralFixup, RegisterFixup, apply_peripheral_fixup
from peripherals.crc import CRC_TYPE_A
from peripherals.exti import EXTI_TYPE_A
from peripherals.fdcan import merge_fdcan

_RCC = PeripheralFixup(registers={
    "CR":        RegisterFixup(reset_value=0b1010101000000),
    "AHBENR":    RegisterFixup(reset_value=0x100),
    "IOPSMENR":  RegisterFixup(use_valid_mask=True),
    "AHBSMENR":  RegisterFixup(use_valid_mask=True),
    "APBSMENR2": RegisterFixup(use_valid_mask=True),
    "ICSCR":     RegisterFixup(unimplemented=True),
})

_FDCAN = PeripheralFixup(registers={
    "CREL":  RegisterFixup(reset_value=0x32141218),
    "ENDN":  RegisterFixup(reset_value=0x87654321),
    "DBTP":  RegisterFixup(reset_value=0xA33),
    "CCCR":  RegisterFixup(reset_value=0x1),
    "NBTP":  RegisterFixup(reset_value=0x06000A03),
    "TOCC":  RegisterFixup(reset_value=0xFFFF0000),
    "TOCV":  RegisterFixup(reset_value=0x0000FFFF),
    "PSR":   RegisterFixup(reset_value=0x0707),
    "XIDAM": RegisterFixup(reset_value=0x1FFFFFFF),
    "TXFQS": RegisterFixup(reset_value=0x00000003),
})

_FLASH = PeripheralFixup(registers={
    "ACR":       RegisterFixup(reset_value=1 << 18 | 1 << 9),
    "CR":        RegisterFixup(reset_value=0xC0000000),
    "PCROP1BSR": RegisterFixup(unimplemented=True),
    "PCROP1BER": RegisterFixup(unimplemented=True),
    "PCROP1AER": RegisterFixup(unimplemented=True),
    "PCROP1ASR": RegisterFixup(unimplemented=True),
})

_SYSCFG = PeripheralFixup(registers={
    "CFGR1": RegisterFixup(fields_unimplemented_except=["MEM_MODE"]),
    "CFGR2": RegisterFixup(unimplemented=True),
    "CFGR3": RegisterFixup(unimplemented=True),
})

class stm32c092xx(STM32Fixups):
    common_periph = {"CRC": "CRC_TYPE_A", "I2C": "I2C_TYPE_A", "IWDG": "IWDG_TYPE_A", "FDCAN": "FDCAN_TYPE_A", "EXTI": "EXTI_TYPE_A", "USART": "USART_TYPE_A"}

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        apply_peripheral_fixup(chip.periph_map, "CRC", CRC_TYPE_A)
        apply_peripheral_fixup(chip.periph_map, "RCC", _RCC)
        apply_peripheral_fixup(chip.periph_map, "FLASH", _FLASH)
        apply_peripheral_fixup(chip.periph_map, "FDCAN", _FDCAN)
        apply_peripheral_fixup(chip.periph_map, "EXTI", EXTI_TYPE_A)
        apply_peripheral_fixup(chip.periph_map, "SYSCFG", _SYSCFG)

    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        merge_fdcan(chip.periph_map)
        # EXTICR[4] is auto-expanded by the parser from the array declaration.
        # IT_LINE_SR[32] is also auto-expanded, but the HAL array name differs
        # from the datasheet naming (ITLINE0..31). Rename here so that the
        # bitfield defines (SYSCFG_ITLINE{N}_SR_*) are captured in phase 2.
        syscfg = chip.periph_map["SYSCFG"]
        for i in range(32):
            old_name = f"IT_LINE_SR{i + 1}"
            if old_name in syscfg:
                reg = syscfg.pop(old_name)
                reg.name = f"ITLINE{i}"
                syscfg[f"ITLINE{i}"] = reg

    @staticmethod
    def post_bitfield_fixups(chip: STM32Chip):
        # ITLINE0..31 were renamed from IT_LINE_SR1..32 in post_register_fixups so
        # the bitfield parser could capture SYSCFG_ITLINE{N}_SR_* defines.
        # Strip the SR_ prefix the HAL uses (removeprefix is literal, unlike lstrip)
        # and mark all registers unimplemented.
        syscfg = chip.periph_map["SYSCFG"]
        for i in range(32):
            reg = syscfg[f"ITLINE{i}"]
            reg.unimplemented = True
            new_fields = {}
            for fname, field in reg.fields.items():
                new_name = fname.removeprefix("SR_")
                field.name = new_name
                new_fields[new_name] = field
            reg.fields = new_fields
